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
#include "httpd.h"

irom void httpd_handle_root(void *arg)
{
	char resp[256];
	char body[] = "You should tell Alexa to discover devices";
	os_sprintf(resp, HTTP_OK_RESP, "text/plain",
			os_strlen(body), body);

	struct espconn *pespconn = (struct espconn *)arg;
	espconn_sent(pespconn, resp, os_strlen(resp));
}

irom void httpd_handle_setupxml(void *arg)
{
	char *resp = (char *)os_zalloc(1024);
	if (resp == NULL) {
		UPNP_INFO("resp mem alloc failed when response setup_xml\r\n");
		return;
	}

	char *body = (char *)os_zalloc(896);
	if (body == NULL) {
		UPNP_INFO("body mem alloc failed when response setup_xml\r\n");
		os_free(resp);
		resp = NULL;
		return;
	}

	struct espconn *pcon = (struct espconn *)arg;
	upnp_dev_t *d = (upnp_dev_t *)(pcon->reverse);
	os_sprintf(body, SETUPXML_BODY, d->dev_voice_name, d->dev_upnp_uuid);

	os_sprintf(resp, HTTP_OK_RESP, "text/xml",
			os_strlen(body), body);

	espconn_sent(pcon, resp, os_strlen(resp));

	UPNP_DEBUG("/setup.xml resp: \r\n%s\r\n", body);

	os_free(body);
	os_free(resp);
	resp = NULL;
	body = NULL;
}

irom void httpd_handle_upnp_ctrl(void *arg, char *data)
{
	struct espconn *pespconn = (struct espconn *)arg;
	upnp_dev_t *d = (upnp_dev_t *)pespconn->reverse;

	char *p = (char *)os_strstr(data, "<BinaryState>");
	if (p != NULL) {
		if (*(p+13) == '1') {
			UPNP_INFO("RX upnp ctrl cmd: Turn on device\r\n");
			d->way_on();
		} else if (*(p+13) == '0') {
			UPNP_INFO("RX upnp ctrl cmd: Turn off device\r\n");
			d->way_off();
		} else {
			UPNP_INFO("RX upnp ctrl cmd is: %c\r\n", *(p+13));
		}
	}

	char *resp = (char *)os_zalloc(128);
	if (resp == NULL) {
		UPNP_INFO("resp mem alloc failed when response upnp ctrl\r\n");
		return;
	}
	os_sprintf(resp, HTTP_OK_RESP, "text/plain", 0, "");

	espconn_sent(pespconn, resp, os_strlen(resp));

	os_free(resp);
	resp = NULL;
}

irom void httpd_handle_evnt_service(void *arg)
{
	UPNP_DEBUG("Handle event service request\r\n");
	char *resp = (char *)os_zalloc(1024);
	if (resp == NULL) {
		UPNP_INFO("resp mem alloc failed when response setup_xml\r\n");
		return;
	}

	os_sprintf(resp, HTTP_OK_RESP, "text/plain",
			os_strlen(EVENT_SERVICE_XML), EVENT_SERVICE_XML);

	struct espconn *pespconn = (struct espconn *)arg;
	espconn_sent(pespconn, resp, os_strlen(resp));

	os_free(resp);
}

irom void httpd_handle_bad_req(void *arg)
{
	char head[128];
	os_sprintf(head, 
			"HTTP/1.0 400 BadRequest\r\n"
			"Content-Length: 0\r\n"
			"Server: lwIP/1.4.0\r\n\r\n");
	int len = os_strlen(head);

	UPNP_INFO("HTTP bad request\r\n");
	struct espconn *pespconn = (struct espconn *)arg;
	espconn_sent(pespconn, head, len);
}

irom void httpd_not_found(void *arg)
{
	char resp[256];
	char body[] = "Not Found :(";
	os_sprintf(resp, 
			"HTTP/1.0 404 Not Found\r\n"
			"Content-Length: %d\r\n"
			"Server: lwIP/1.4.0\r\n"
			"Content-type: text/plain\r\n"
			"Connection: close\r\n\r\n%s",
			os_strlen(body), body);

	struct espconn *pespconn = (struct espconn *)arg;
	espconn_sent(pespconn, resp, os_strlen(resp));
}

irom void tcp_srv_recv_cb(void *arg, char *data, uint16_t len)
{
	char *d = data;

	if (strncmp(d, "GET / ", 6) == 0 ) {
		httpd_handle_root(arg);
	} else if (strncmp(d, "GET /setup.xml ", 15) == 0) {
		httpd_handle_setupxml(arg);
	} else if (strncmp(d, "POST /upnp/control/basicevent1 ", 31) == 0) {
		httpd_handle_upnp_ctrl(arg, data);
	} else if (strncmp(d, "GET /eventservice.xml ", 22) == 0) {
		httpd_handle_evnt_service(arg);
	} else {
		UPNP_INFO("------- TCP recv : -------\r\n%s\r\n------------------\r\n", data);
		httpd_not_found(arg);
	}
}

irom void tcp_srv_sent_cb(void *arg)
{
	//data sent successfully
	UPNP_INFO("TCP sent success\r\n");
}

irom void tcp_srv_discon_cb(void *arg)
{
	//tcp disconnect successfully
	UPNP_INFO("TCP disconnect succeed\r\n");
}

irom void tcp_srv_recon_cb(void *arg, sint8 err)
{
	//error occured , tcp connection broke.
	UPNP_INFO("TCP reconnect callback, error code %d\r\n", err);
}

irom void tcp_srv_listen(void *arg)
{
	struct espconn *pesp_conn = (struct espconn *)arg;
	UPNP_DEBUG("TCP is listening ... \r\n");

	espconn_regist_recvcb(pesp_conn, tcp_srv_recv_cb);
	espconn_regist_reconcb(pesp_conn, tcp_srv_recon_cb);
	espconn_regist_disconcb(pesp_conn, tcp_srv_discon_cb);

	espconn_regist_sentcb(pesp_conn, tcp_srv_sent_cb);
}

int httpd_start(upnp_dev_t *d)
{
	if (d->esp_conn == NULL) {
		d->esp_conn = (struct espconn *)os_zalloc(sizeof(struct espconn));
		// Check memory
		if (d->esp_conn == NULL) {
			UPNP_INFO("%s: not enough memory\r\n", __func__);
			return -1;
		}
	}
	d->esp_conn->type = ESPCONN_TCP;
	d->esp_conn->state = ESPCONN_NONE;
	d->esp_conn->proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));
	if (d->esp_conn->proto.tcp == NULL) {
		UPNP_INFO("%s: not enough memory, alloc for esp_tcp failed\r\n", __func__);
		return -1;
	}	

	d->esp_conn->reverse = (void *)d;
	d->esp_conn->proto.tcp->local_port = d->port;

	espconn_regist_connectcb(d->esp_conn, tcp_srv_listen);
	uint8_t ret = espconn_accept(d->esp_conn);
	UPNP_DEBUG("espconn_accept [%d] !!! \r\n", ret);
	return ret;
}

irom void httpd_stop(upnp_dev_t *d)
{
	if (d->esp_conn != NULL) {
		espconn_delete(d->esp_conn);

		if (d->esp_conn->proto.tcp != NULL) {
			os_free(d->esp_conn->proto.tcp);
			d->esp_conn->proto.tcp = NULL;
		}

		os_free(d->esp_conn);
		d->esp_conn = NULL;
	}
}
