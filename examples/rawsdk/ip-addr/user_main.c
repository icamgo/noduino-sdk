/*
 *  Copyright (c) 2015 - 2025 MaiKe Labs
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

#include "driver/uart.h"

#include "ip_addr.h"

static volatile os_timer_t hello_timer;

void hello_timerfunc(void *arg)
{
	ip_addr_t ip;
	ip.addr = 0x0100000A;


	os_printf("%s\r\n", "Testing ip_addr...");

	os_printf("ip.addr = 0x%08X\r\n", ip.addr);

	//os_printf("IP = " IPSTR "\r\n", IP2STR(&(ip.addr)));
	os_printf("IP = " IPSTR "\r\n", IP2STR(&ip));

	if (ip.addr == 0x0100000A) {

		os_printf("Hahahahaha, it's OK\r\n");
	}
}

//user_init is the user entry point of the Espressif SDK
void ICACHE_FLASH_ATTR user_init()
{
	//Initialize the uart0 and uart1 in 115200 bitrate
	uart_init(BIT_RATE_115200, BIT_RATE_115200);

	//disable the wifi
	wifi_set_opmode(NULL_MODE);

	//Disable the timer
	os_timer_disarm(&hello_timer);

	//Setup timer
	os_timer_setfn(&hello_timer, (os_timer_func_t *) hello_timerfunc, NULL);

	//enable the timer
	//&hello_timer is the pointer
	//2000 is the fire time in ms
	//0 for once and 1 for repeating
	os_timer_arm(&hello_timer, 2000, 1);
}
