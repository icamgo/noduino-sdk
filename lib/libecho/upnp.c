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
#include "ssdp.h"
#include "upnp.h"

static udp_srv_t *udps = NULL;

static upnp_dev_t *devs_array = NULL;
static uint8_t devs_len = 0;

static void dev_uuid_init(upnp_dev_t devs[], uint32_t ways)
{
	int i;
	char mac[6];
	wifi_get_macaddr(STATION_IF, mac);

	for (i = 0; i < ways; i++) {
		os_memset(devs[i].dev_upnp_uuid, 0, 64);
		os_sprintf(devs[i].dev_upnp_uuid, "38323636-4558-4dda-9188-cda0%02x%02x%02x%02x",
					mac[3], mac[4], mac[5], devs[i].port);

	}
}

char *get_dev_target(dev_type_t tp)
{
	switch(tp) {
		case WEMO_SWITCH:
			return "urn:Belkin:device:controllee:1";
			break;
		case WEMO_LIGHTSWITCH:
			return "urn:Belkin:device:lightswitch:1";
			break;
		case WEMO_INSIGHT:
			return "urn:Belkin:device:insight:1";
		case WEMO_BULB:
			return "urn:Belkin:device:bridge:1";
			break;
		case WEMO_MOTION:
			return "urn:Belkin:device:sensor:1";
			break;
		case WEMO_MAKER:
			return "urn:Belkin:device:Maker:1";
			break;
		case HUE:
			return "urn:schemas-upnp-org:device:basic:1";
			//return "upnp:rootdevice";
			break;
		default:
			return NULL;
	}
}

void ssdp_gen_resp_pkt(char *pb, upnp_dev_t *pd, char *myip)
{
	char *name, *num;

	if (pd->model_name != NULL)
		name = pd->model_name;
	else
		name = "Unspecified";

	if (pd->model_num != NULL)
		num = pd->model_num;
	else
		num = "1.0";

	char *p = "hue-bridgeid: 929000226503";

	char *st = pd->dev_type != HUE ?
			"urn:Belkin:device:**":get_dev_target(pd->dev_type);

	os_sprintf(pb, SSDP_PKT,
			SSDP_RESP,
			name,
			num,
			pd->dev_upnp_uuid,
			"ST",
			st,
			myip, pd->port,
			pd->dev_type == HUE ? p : "");
}

void ssdp_resp(upnp_dev_t *dev)
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

	char *pkt_buf = (char *)os_zalloc(512);
	if (pkt_buf == NULL) {
		UPNP_INFO("pkt_buf mem alloc failed\r\n");
		return;
	}

	char myip[16];
	os_sprintf(myip, IPSTR, IP2STR(&(ipconfig.ip)));
	UPNP_DEBUG("my ip is %s\r\n", myip);

	ssdp_gen_resp_pkt(pkt_buf, dev, myip);

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

void ssdp_process_req()
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

			char *x = NULL;
			switch (devs_array[0].dev_type) {
				case WEMO_SWITCH:
				case WEMO_INSIGHT:
				case WEMO_LIGHTSWITCH:
				case WEMO_BULB:
				case WEMO_MOTION:
				case WEMO_MAKER:
					x = (char *)os_strstr(p, "urn:Belkin:device:");
					break;
				case HUE:
					x = (char *)os_strstr(p, "urn:schemas-upnp-org:device:");
					break;
			}
			if (NULL != x) {
				UPNP_INFO("Received request of discovering %s\r\n",
						get_dev_target(devs_array[0].dev_type));
				int i;
				for (i = 0; i < devs_len; i++) {
					ssdp_resp(&devs_array[i]);
					os_delay_us(10000);
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

	dev_uuid_init(devs, ways);

	ip_addr_t multicast_addr;
	multicast_addr.addr = UDP_SRV_IP;

    if (igmp_joingroup(&(ipconfig.ip), &multicast_addr)!= ERR_OK) {
		UPNP_INFO("IGMP join group failed\r\n");
        return -2;
    }

	devs_array = devs;
	devs_len = ways;
	udp_srv_set_rx_handler(udps, ssdp_process_req);

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
