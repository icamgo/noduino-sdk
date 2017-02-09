/*
 *  Copyright (c) 2016 - 2025 MaiKe Labs
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
*/
#include "upnp.h"

static udp_srv_t *udps = NULL;

static upnp_dev_t *devs_array = NULL;
static uint8_t devs_len = 0;

void upnp_dev_uuid_init(upnp_dev_t devs[], uint32_t ways)
{
	int i;
	char mac[6];
	wifi_get_macaddr(STATION_IF, mac);

	for (i = 0; i < ways; i++) {
		os_memset(devs[i].dev_upnp_uuid, 0, 64);
		os_sprintf(devs[i].dev_upnp_uuid, "Socket-1_0-38323636-4558-4dda-9188-cda0%02x%02x%02x%02x",
					mac[3], mac[4], mac[5], devs[i].port);

	}
}

void upnp_ssdp_resp(char *dev_upnp_uuid, uint32_t port)
{
	UPNP_DEBUG("Respose the request of ssdp discover\r\n");

	struct ip_info ipconfig;
	wifi_get_ip_info(STATION_IF, &ipconfig);

	if (ipconfig.ip.addr == 0) {
		UPNP_INFO("ipaddr of dev is 0\r\n");
		return ;
	}

	ip_addr_t rip;
	rip.addr = udp_srv_get_remote_ip(udps);
	uint16_t rport = udp_srv_get_remote_port(udps);

	bool rt = udp_srv_connect(udps, rip, rport);
	if (!rt) {
		UPNP_INFO("UDP connect failed\r\n");
		return;
	}

	char myip[16];
	os_sprintf(myip, IPSTR, IP2STR(&(ipconfig.ip)));
	UPNP_DEBUG("my ip is %s\r\n", myip);

	char *pkt_buf = (char *)os_zalloc(512);
	if (pkt_buf == NULL) {
		UPNP_INFO("pkt_buf mem alloc failed\r\n");
		return;
	}

	os_sprintf(pkt_buf, SSDP_DISCOVER_RESP, myip, port, dev_upnp_uuid);

	UPNP_DEBUG("pkt_buf (%d): %s\r\n", os_strlen(pkt_buf), pkt_buf);

	// size_t 
	udp_srv_append(udps, pkt_buf, os_strlen(pkt_buf));

	os_free(pkt_buf);
	pkt_buf = NULL;

	// bool
	rt = udp_srv_send(udps, &rip, rport);
	if(!rt) {
		UPNP_INFO("UDP send failed\r\n");
	}
}

void upnp_process_ssdp_req()
{
	if (!udp_srv_next(udps)) {
		UPNP_INFO("There is no udp data received\r\n");
		return;
	}
		
	size_t sz = udp_srv_get_size(udps);
	if (sz > 0) {

#ifdef DEBUG
		ip_addr_t rip;
		rip.addr = udp_srv_get_remote_ip(udps);
		uint16_t rport = udp_srv_get_remote_port(udps);

		UPNP_DEBUG("Received packet of size %d from "IPSTR":%d\r\n", sz,
				IP2STR(&rip),
				rport);
#endif
		char *rx_buf = (char *)os_zalloc(255);
		if (rx_buf == NULL) {
			UPNP_INFO("rx_buf mem alloc failed\r\n");
			return;
		}
		size_t rs = 0;

		rs = udp_srv_read(udps, rx_buf, 255);
		if (rs > 0) {
			UPNP_DEBUG("read %d bytes udp data\r\n", rs);
			rx_buf[rs] = '\0';
		}

		// parse the rx_buf, if "urn:Belkin:device:**", response
		char *p = (char *)os_strstr(rx_buf, "M-SEARCH ");

		if (p != NULL) {
			UPNP_DEBUG("Received the SSDP discover packet\r\n");
			UPNP_DEBUG("-------------------\r\n");
			UPNP_DEBUG("%s\r\n", rx_buf);
			UPNP_DEBUG("-------------------\r\n");
			char *x = (char *)os_strstr(p, "urn:Belkin:device:");
			if (NULL != x) {
				UPNP_INFO("Received request of discovering Belkin device\r\n");
				int i;
				for (i = 0; i < devs_len; i++) {
					upnp_ssdp_resp(devs_array[i].dev_upnp_uuid,
							devs_array[i].port);
				}
			}
		}

		os_free(rx_buf);
		rx_buf = NULL;
	}
}

int upnp_start(upnp_dev_t *devs, int ways)
{
	if (udps == NULL) {
		udps = udp_srv_create();
		if (udps == NULL) {
			UPNP_INFO("create udp server failed\r\n");
			return -4;
		}
	}

	struct ip_info ipconfig;
	wifi_get_ip_info(STATION_IF, &ipconfig);

	if (ipconfig.ip.addr == 0) {
		UPNP_INFO("ipaddr of dev is 0\r\n");
		return -1;
	}

	upnp_dev_uuid_init(devs, ways);

	ip_addr_t multicast_addr;
	multicast_addr.addr = UDP_SRV_IP;

    if (igmp_joingroup(&(ipconfig.ip), &multicast_addr)!= ERR_OK) {
		UPNP_INFO("IGMP join group failed\r\n");
        return -2;
    }

	devs_array = devs;
	devs_len = ways;
	udp_srv_set_rx_handler(udps, upnp_process_ssdp_req);

    if (!udp_srv_listen(udps, *IP_ADDR_ANY, UDP_SRV_PORT)) {
		UPNP_INFO("UDP listen failed\r\n");
        return -3;
    }

	// init the web server
	int i;
	for (i = 0; i < ways; i++) {
		httpd_start(&devs[i]);
	}

	return 0;
}

void upnp_stop(upnp_dev_t *devs, int ways)
{
	int i;
	for (i = 0; i < ways; i++) {
		httpd_stop(&devs[i]);
	}
}
