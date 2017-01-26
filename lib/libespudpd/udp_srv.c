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
#include "udp_srv.h"

#define GET_IP_HDR(pb) ((struct ip_hdr*)(((uint8_t*)((pb)->payload)) - UDP_HLEN - IP_HLEN))
#define GET_UDP_HDR(pb) ((struct udp_hdr*)(((uint8_t*)((pb)->payload)) - UDP_HLEN))

irom udp_srv_t *udp_srv_create()
{
	udp_srv_t *srv = (udp_srv_t *) os_zalloc(sizeof(udp_srv_t));
	if (srv == NULL) {
		INFO("alloc struct udp_srv failed\r\n");
		return NULL;
	}

	srv->_pcb = udp_new();
	srv->_rx_buf = 0;
	srv->_first_buf_taken = false;
	srv->_rx_buf_offset = 0;
	srv->_tx_buf_head = 0;
	srv->_tx_buf_cur = 0;
	srv->_tx_buf_offset = 0;

#ifdef LWIP_MAYBE_XCC
	srv->_mcast_ttl = 1;
#endif
}

irom void udp_srv_delete(udp_srv_t * srv)
{
	udp_remove(srv->_pcb);
	srv->_pcb = 0;

	if (srv->_tx_buf_head) {
		pbuf_free(srv->_tx_buf_head);
		srv->_tx_buf_head = 0;
		srv->_tx_buf_cur = 0;
		srv->_tx_buf_offset = 0;
	}

	if (srv->_rx_buf) {
		pbuf_free(srv->_rx_buf);
		srv->_rx_buf = 0;
		srv->_rx_buf_offset = 0;
	}

	os_free(srv);
}

irom bool udp_srv_connect(udp_srv_t * srv, ip_addr_t addr, uint16_t port)
{
	ip_addr_copy(srv->_pcb->remote_ip, addr);
	srv->_pcb->remote_port = port;
	return true;
}

irom void udp_srv_recv(void *arg, struct udp_pcb *upcb, struct pbuf *pb,
		       ip_addr_t * addr, u16_t port)
{
	udp_srv_t *srv = (udp_srv_t *) arg;
	if (srv->_rx_buf) {
		// there is some unread data
		// chain the new pbuf to the existing one
		INFO(":urch %d, %d\r\n", srv->_rx_buf->tot_len, pb->tot_len);
		pbuf_cat(srv->_rx_buf, pb);
	} else {
		INFO(":urn %d\r\n", pb->tot_len);
		srv->_first_buf_taken = false;
		srv->_rx_buf = pb;
		srv->_rx_buf_offset = 0;
	}
	if (srv->_on_rx) {
		srv->_on_rx();
	}
}

irom bool udp_srv_listen(udp_srv_t * srv, ip_addr_t addr, uint16_t port)
{
	udp_recv(srv->_pcb, udp_srv_recv, (void *)srv);
	err_t err = udp_bind(srv->_pcb, &addr, port);
	return err == ERR_OK;
}

irom void udp_srv_disconnect(udp_srv_t * srv)
{
	udp_disconnect(srv->_pcb);
}

irom void udp_srv_set_multicast_interface(udp_srv_t * srv, ip_addr_t addr)
{
	udp_set_multicast_netif_addr(srv->_pcb, addr);
}

irom void udp_srv_set_multicast_ttl(udp_srv_t * srv, int ttl)
{
#ifdef LWIP_MAYBE_XCC
	srv->_mcast_ttl = ttl;
#else
	udp_set_multicast_ttl(srv->_pcb, ttl);
#endif
}

// warning: handler is called from tcp stack context
// esp_yield and non-reentrant functions which depend on it will fail
irom void udp_srv_set_rx_handler(udp_srv_t * srv, rxhandler_fun handler)
{
	srv->_on_rx = handler;
}

irom size_t udp_srv_get_size(udp_srv_t * srv)
{
	if (!srv->_rx_buf)
		return 0;

	return srv->_rx_buf->len - srv->_rx_buf_offset;
}

irom uint32_t udp_srv_get_remote_ip(udp_srv_t * srv)
{
	if (!srv->_rx_buf)
		return 0;

	struct ip_hdr *iphdr = GET_IP_HDR(srv->_rx_buf);
	if (iphdr != NULL)
		return iphdr->src.addr;
	else
		return 0;
}

irom uint16_t udp_srv_get_remote_port(udp_srv_t * srv)
{
	if (!srv->_rx_buf)
		return 0;

	struct udp_hdr *udphdr = GET_UDP_HDR(srv->_rx_buf);
	if (udphdr != NULL)
		return ntohs(udphdr->src);
	else
		return 0;
}

irom uint32_t udp_srv_get_dest_ip(udp_srv_t * srv)
{
	struct ip_hdr *iphdr = GET_IP_HDR(srv->_rx_buf);
	if (iphdr != NULL)
		return iphdr->dest.addr;
	else
		return 0;
}

irom uint16_t udp_srv_get_local_port(udp_srv_t * srv)
{
	if (!srv->_pcb)
		return 0;

	return srv->_pcb->local_port;
}

irom bool udp_srv_next(udp_srv_t * srv)
{
	if (!srv->_rx_buf)
		return false;

	if (!srv->_first_buf_taken) {
		srv->_first_buf_taken = true;
		return true;
	}

	struct pbuf *head = srv->_rx_buf;
	srv->_rx_buf = srv->_rx_buf->next;
	srv->_rx_buf_offset = 0;

	if (srv->_rx_buf) {
		pbuf_ref(srv->_rx_buf);
	}
	pbuf_free(head);
	return srv->_rx_buf != 0;
}

irom int udp_srv_read_byte(udp_srv_t * srv)
{
	if (!srv->_rx_buf || srv->_rx_buf_offset == srv->_rx_buf->len)
		return -1;

	char *d = (char *)(srv->_rx_buf->payload);
	char c = d[srv->_rx_buf_offset];

	udp_srv_consume(srv, 1);
	return c;
}

irom size_t udp_srv_read(udp_srv_t * srv, char *dst, size_t size)
{
	if (!srv->_rx_buf)
		return 0;

	size_t max_size = srv->_rx_buf->len - srv->_rx_buf_offset;
	size = (size < max_size) ? size : max_size;
	INFO(":urd %d, %d, %d\r\n", size, srv->_rx_buf->len,
	     srv->_rx_buf_offset);

	memcpy(dst, (char *)(srv->_rx_buf->payload) + srv->_rx_buf_offset,
	       size);
	udp_srv_consume(srv, size);

	return size;
}

irom int udp_srv_peek(udp_srv_t * srv)
{
	if (!srv->_rx_buf || srv->_rx_buf_offset == srv->_rx_buf->len)
		return -1;

	char *d = (char *)srv->_rx_buf->payload;

	return d[srv->_rx_buf_offset];
}

irom void udp_srv_flush(udp_srv_t * srv)
{
	if (!srv->_rx_buf)
		return;

	udp_srv_consume(srv, srv->_rx_buf->len - srv->_rx_buf_offset);
}

irom size_t udp_srv_append(udp_srv_t * srv, const char *data, size_t size)
{
	if (!srv->_tx_buf_head ||
	    srv->_tx_buf_head->tot_len < srv->_tx_buf_offset + size) {
		udp_srv_reserve(srv, srv->_tx_buf_offset + size);
	}

	size_t left_to_copy = size;
	while (left_to_copy) {
		// size already used in current pbuf
		size_t used_cur = srv->_tx_buf_offset -
		    (srv->_tx_buf_head->tot_len - srv->_tx_buf_cur->tot_len);
		size_t free_cur = srv->_tx_buf_cur->len - used_cur;

		if (free_cur == 0) {
			srv->_tx_buf_cur = srv->_tx_buf_cur->next;
			continue;
		}

		size_t will_copy =
		    (left_to_copy < free_cur) ? left_to_copy : free_cur;

		memcpy((char *)(srv->_tx_buf_cur->payload) + used_cur, data,
		       will_copy);

		srv->_tx_buf_offset += will_copy;
		left_to_copy -= will_copy;
		data += will_copy;
	}
	return size;
}

irom bool udp_srv_send(udp_srv_t * srv, ip_addr_t * addr, uint16_t port)
{
	size_t data_size = srv->_tx_buf_offset;

	struct pbuf *tx_copy = pbuf_alloc(PBUF_TRANSPORT, data_size, PBUF_RAM);

	if (tx_copy == NULL) {
		INFO("pbuf alloc failed\r\n");
		return false;
	}

	uint8_t *dst = (uint8_t *) (tx_copy->payload);

	struct pbuf *p;
	for (p = srv->_tx_buf_head; p; p = p->next) {
		size_t will_copy = (data_size < p->len) ? data_size : p->len;
		memcpy(dst, p->payload, will_copy);
		dst += will_copy;
		data_size -= will_copy;
	}

	pbuf_free(srv->_tx_buf_head);
	srv->_tx_buf_head = 0;
	srv->_tx_buf_cur = 0;
	srv->_tx_buf_offset = 0;

	if (!addr) {
		addr = &(srv->_pcb->remote_ip);
		port = srv->_pcb->remote_port;
	}
#ifdef LWIP_MAYBE_XCC
	uint16_t old_ttl = srv->_pcb->ttl;
	if (ip_addr_ismulticast(addr)) {
		srv->_pcb->ttl = srv->_mcast_ttl;
	}
#endif
	err_t err = udp_sendto(srv->_pcb, tx_copy, addr, port);
	if (err != ERR_OK) {
		INFO(":ust rc=%d\r\n", err);
	}
#ifdef LWIP_MAYBE_XCC
	srv->_pcb->ttl = old_ttl;
#endif
	pbuf_free(tx_copy);
	return err == ERR_OK;
}

irom static void udp_srv_reserve(udp_srv_t * srv, size_t size)
{
	const size_t pbuf_unit_size = 128;
	if (!srv->_tx_buf_head) {
		srv->_tx_buf_head =
		    pbuf_alloc(PBUF_TRANSPORT, pbuf_unit_size, PBUF_RAM);
		srv->_tx_buf_cur = srv->_tx_buf_head;
		srv->_tx_buf_offset = 0;
	}

	size_t cur_size = srv->_tx_buf_head->tot_len;
	if (size < cur_size)
		return;

	size_t grow_size = size - cur_size;

	while (grow_size) {
		struct pbuf *pb =
		    pbuf_alloc(PBUF_TRANSPORT, pbuf_unit_size, PBUF_RAM);
		pbuf_cat(srv->_tx_buf_head, pb);
		if (grow_size < pbuf_unit_size)
			return;
		grow_size -= pbuf_unit_size;
	}
}

irom static void udp_srv_consume(udp_srv_t * srv, size_t size)
{
	srv->_rx_buf_offset += size;
}
