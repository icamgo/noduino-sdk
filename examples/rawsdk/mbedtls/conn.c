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
#include "osapi.h"
#include "user_interface.h"
#include "espconn.h"

#include "conn.h"

static char sendbuff[] = HTTP_REQUEST;
static os_timer_t user_dns_timer;	// user_http_timer;
static struct espconn user_tcp_conn;
static struct _esp_tcp user_tcp;
static ip_addr_t tcp_server_ip;

static void ICACHE_FLASH_ATTR user_conn_server(struct espconn *pespconn)
{
	pespconn->proto.tcp->local_port = espconn_port();
	INFO("espconn connect return %d ! \r\n",
		espconn_secure_connect(pespconn));
}

static void ICACHE_FLASH_ATTR user_send_data(struct espconn *pespconn)
{
	INFO("send data return %d ! \r\n",
		espconn_secure_send(pespconn, sendbuff, os_strlen(sendbuff)));
}

static void ICACHE_FLASH_ATTR
user_dns_found(const char *name, ip_addr_t * ipaddr, void *arg)
{
	struct espconn *pespconn = arg;
	if (ipaddr && ipaddr->addr)	//dns got ip succeed
	{
		INFO("user_dns_found %d.%d.%d.%d \r\n",
			*((uint8 *) & ipaddr->addr),
			*((uint8 *) & ipaddr->addr + 1),
			*((uint8 *) & ipaddr->addr + 2),
			*((uint8 *) & ipaddr->addr + 3));
		os_printf("heap %d\n", system_get_free_heap_size());
		tcp_server_ip.addr = ipaddr->addr;
		memcpy(pespconn->proto.tcp->remote_ip, &ipaddr->addr, 4);
		os_timer_disarm(&user_dns_timer);
		user_conn_server(pespconn);
	} else {
		INFO("user_dns_found NULL \r\n");
	}
}

static void ICACHE_FLASH_ATTR user_dns_check_cb(void *arg)
{
	struct espconn *pespconn = arg;
	if (tcp_server_ip.addr) {
		os_timer_disarm(&user_dns_timer);
	} else {
		espconn_gethostbyname(pespconn, SERVER_DOMAIN, &tcp_server_ip, user_dns_found);
	}
}

static void ICACHE_FLASH_ATTR
user_tcp_recv_cb(void *arg, char *pusrdata, unsigned short length)
{
	//received some data from tcp connection
	INFO("tcp recv ! length=%d\r\n", length);
	INFO("%s\n", pusrdata);
}

static void ICACHE_FLASH_ATTR user_tcp_sent_cb(void *arg)
{
	//data sent successfully
	INFO("tcp sent succeed ! \r\n");
}

static void ICACHE_FLASH_ATTR user_tcp_discon_cb(void *arg)
{
	//tcp disconnect successfully
	INFO("tcp disconnect succeed %d! \r\n", system_get_free_heap_size());
}

static void ICACHE_FLASH_ATTR user_tcp_recon_cb(void *arg, sint8 err)
{
	//error occured , tcp connection broke. user can try to reconnect here.
	INFO("reconnect callback, error code %d ! \r\n", err);
}

static void ICACHE_FLASH_ATTR user_tcp_connect_cb(void *arg)
{
	struct espconn *pespconn = arg;
	INFO("connect succeed ! \r\n");
	espconn_regist_recvcb(pespconn, user_tcp_recv_cb);
	espconn_regist_sentcb(pespconn, user_tcp_sent_cb);

	user_send_data(pespconn);
}

void ICACHE_FLASH_ATTR user_conn_init(void)
{
	const char esp_tcp_server_ip[4] = { 192, 168, 1, 101 };
	tcp_server_ip.addr = SERVER_IP;
	memcpy(user_tcp.remote_ip, &tcp_server_ip.addr, 4);
	user_tcp.remote_port = SERVER_PORT;
	user_tcp_conn.proto.tcp = &user_tcp;
	user_tcp_conn.type = ESPCONN_TCP;
	user_tcp_conn.state = ESPCONN_NONE;
	espconn_secure_set_size(ESPCONN_CLIENT, 3072);
//	espconn_secure_ca_enable(ESPCONN_CLIENT, 0x50);
//	espconn_secure_cert_req_enable(ESPCONN_CLIENT,0x51);
	espconn_regist_connectcb(&user_tcp_conn, user_tcp_connect_cb);
	espconn_regist_disconcb(&user_tcp_conn, user_tcp_discon_cb);
	espconn_regist_reconcb(&user_tcp_conn, user_tcp_recon_cb);
#if 1
	os_timer_disarm(&user_dns_timer);
	os_timer_setfn(&user_dns_timer, (os_timer_func_t *) user_dns_check_cb,
		       &user_tcp_conn);
	os_timer_arm(&user_dns_timer, 2000, 1);
#else
	os_memcpy(&user_tcp_conn.proto.tcp->remote_ip, esp_tcp_server_ip, 4);
	user_tcp_conn.proto.tcp->remote_port = 443;
	user_conn_server(&user_tcp_conn);
#endif
}
