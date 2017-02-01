/*
 *  Copyright (c) 2014 Ivan Grokhotkov. All rights reserved.
 *  Copyright (c) 2016 - 2025 MaiKe Labs
 *
 *  Ported from the esp8266 core for Arduino environment.
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
#ifndef __UDP_SRV_H__
#define __UDP_SRV_H__

#include "user_interface.h"

#include "lwip/netif.h"
#include "lwip/inet.h"
#include "netif/etharp.h"
#include "lwip/udp.h"                                                  
#include "lwip/ip.h"
#include "lwip/init.h"
#include "lwip/opt.h"                                                  
#include "lwip/igmp.h"                                                 
#include "lwip/mem.h"
#include "ets_sys.h"
#include "os_type.h"

#include "user_config.h"

typedef void (*rxhandler_fun)(void);

typedef struct udp_srv {
	struct udp_pcb* _pcb;
	struct pbuf* _rx_buf;
	bool _first_buf_taken;
	size_t _rx_buf_offset;
	struct pbuf* _tx_buf_head;
	struct pbuf* _tx_buf_cur;
	size_t _tx_buf_offset;
	rxhandler_fun _on_rx;
#ifdef LWIP_MAYBE_XCC
	uint16_t _mcast_ttl
#endif
} udp_srv_t;

udp_srv_t* udp_srv_create();
void udp_srv_delete(udp_srv_t *srv);

// UDP rx
bool udp_srv_listen(udp_srv_t *srv, ip_addr_t addr, uint16_t port);
void udp_srv_set_rx_handler(udp_srv_t *srv, rxhandler_fun handler);

// UDP tx
bool udp_srv_connect(udp_srv_t *srv, ip_addr_t addr, uint16_t port);
size_t udp_srv_append(udp_srv_t *srv, const char* data, size_t size);
bool udp_srv_send(udp_srv_t *srv, ip_addr_t* addr, uint16_t port);

void udp_srv_disconnect(udp_srv_t *srv);

void udp_srv_set_multicast_interface(udp_srv_t *srv, ip_addr_t addr);
void udp_srv_set_multicast_ttl(udp_srv_t *srv, int ttl);

uint32_t udp_srv_get_remote_ip(udp_srv_t *srv);
uint16_t udp_srv_get_remote_port(udp_srv_t *srv);
uint32_t udp_srv_get_dest_ip(udp_srv_t *srv);
uint16_t udp_srv_get_local_port(udp_srv_t *srv);

size_t udp_srv_get_size(udp_srv_t *srv);
bool udp_srv_next(udp_srv_t *srv);

size_t udp_srv_read(udp_srv_t *srv, char* dst, size_t size);
int udp_srv_read_byte(udp_srv_t *srv);

int udp_srv_peek(udp_srv_t *srv);
void udp_srv_flush(udp_srv_t *srv);

static void udp_srv_reserve(udp_srv_t *srv, size_t size);
static void udp_srv_consume(udp_srv_t *srv, size_t size);

#endif
