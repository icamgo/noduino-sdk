/*
 *  Copyright (c) 2019 - 2029 MaiKe Labs
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

#include "quark-nb.h"

#define POST_FMT	"POST /dev/quark-nb HTTP/1.0\r\n\
Accept: */*\r\n\
Content-Length: %d\r\n\
Content-Type: text/html\r\n\
Connection: close\r\n\r\n%s\r\n"

#define	PKG_FMT		"\!U/%.2f/P/%d/devid/%s/rssi/%d"

char tx_buf[512];
char rx_buf[256];

struct in_addr remote_addr;
uint16_t srv_port;

void push_data_via_tcp()
{
	struct sockaddr_in srv_addr;  
	int sock_fd;  
	int data_len;

	char body[64];
	char imei[16];
	int rssi = 0, rxlevel = 0;
	float vbat = 0.0;
	long press = 0;

	vbat = get_vbat();

	sensor_power_on();
	vTaskDelay(1);

	press = bmp180_get_press();

	sensor_power_off();

	memset(imei, 0, 16);
	opencpu_get_imei(imei);

	opencpu_csq(&rssi, &rxlevel);

	inet_aton("182.92.5.106", &remote_addr);  
	srv_port = 80;

	sock_fd = socket(AF_INET, SOCK_STREAM, 0);  

	if (sock_fd == -1) {  
		opencpu_printf ("Socket create error\n");
		 return;
	}  

	memset(&srv_addr, 0, sizeof(srv_addr));

	srv_addr.sin_family = AF_INET;  
	srv_addr.sin_addr.s_addr = remote_addr.s_addr;
	srv_addr.sin_port = htons(srv_port);

	if(connect(sock_fd, (struct sockaddr *)&srv_addr, sizeof(struct sockaddr))) {
		opencpu_printf ("Tcp connect error\n");
		return;
	}

	memset(body, 0, 64);
	sprintf(body, PKG_FMT, vbat, press, imei, rssi);

	opencpu_printf("%s\r\n", body);

	memset(tx_buf, 0, 512);
	sprintf(tx_buf, POST_FMT, strlen(body), body);

	opencpu_printf ("Tcp data sending\n");
	send(sock_fd, (char *)tx_buf, sizeof(tx_buf), 0);

	memset(rx_buf, 0, 256);

	opencpu_printf ("Tcp waiting...\n");
	data_len = recv(sock_fd, rx_buf, 256, MSG_TRUNC);
	opencpu_printf ("Waiting end\n");

	if(data_len >0) {
		opencpu_printf ("Get: %s\n", rx_buf);
		opencpu_printf ("Len: %d\n", data_len);			 
	}

	close(sock_fd);
}

void push_data_via_udp()
{
	struct sockaddr_in srv_addr;  
	int sock_fd; 
	struct addrinfo dd;
	struct sockaddr_in from;
	int data_len;

	inet_aton("182.92.5.106", &remote_addr);  
	srv_port = 8081;	

	socklen_t fromlen = sizeof(struct sockaddr_in);
	sock_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if(sock_fd == -1) {
		opencpu_printf ("Socket create error\n");
		return;
	}
	memset(&srv_addr, 0, sizeof(srv_addr)); 
	srv_addr.sin_family = AF_INET;  
	srv_addr.sin_addr.s_addr = remote_addr.s_addr;  
	srv_addr.sin_port = htons(srv_port);  

	connect(sock_fd, (struct sockaddr *)&srv_addr, sizeof(struct sockaddr)); 

	opencpu_printf ("Data sending\n");

	send(sock_fd, (char *)POST_FMT, sizeof(POST_FMT), 0);

	memset(rx_buf, 0, 256);
	/*data_len = recvfrom(sock_fd, rx_buf,
					100, MSG_TRUNC | MSG_DONTWAIT, (struct sockaddr*)&from, &fromlen);*/
	opencpu_printf ("Waiting...\n");

	data_len = recvfrom(sock_fd, rx_buf, 256,
					MSG_TRUNC, (struct sockaddr*)&from, &fromlen);

	opencpu_printf("Waiting end\n");				   

	if(data_len > 0) {
		opencpu_printf ("Get: %s\r\n", rx_buf);
		opencpu_printf ("Len: %d\r\n", data_len);			 
	}
		
	close(sock_fd);	
}

//dns功能回调函数
void test_cmdns_cb(unsigned char *ip)
{
	opencpu_printf ("OpenCPU DNS: %s\r\n", ip);	
}

//发起dns测试的函数
void test_dns()
{
	opencpu_printf ("Result: %d\n", opencpu_get_host_by_name("api.noduino.org", 0, test_cmdns_cb));
}
