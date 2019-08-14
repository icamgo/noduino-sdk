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

#define OPENCPU_MAIN_UART HAL_UART_0

unsigned char uart_cmd = 0;

static void user_uart_callback(hal_uart_callback_event_t status, void *user_data)
{
	char buffer[64];
	char *pbuf;
	pbuf = buffer;
	int temp1;
	if(status == HAL_UART_EVENT_READY_TO_READ)
	{
		memset(buffer,0,64);
		temp1 = opencpu_uart_receive(OPENCPU_MAIN_UART, pbuf, 64);
		// opencpu_printf("get:%d\n",temp1);
		opencpu_printf("%s",pbuf);
		uart_cmd = pbuf[0];
	}
}

void opencpu_printf (const char *str, ...)
{

	/*
	 * This needs to be large enough to store the string
	 * TODO: Change magic number
	*/
	static unsigned char s[600];

	int i;
	unsigned char *p;
	va_list args;
	int str_len;

	if((str == NULL) || (strlen(str) == 0)) {
		return;
	}
	va_start(args, str);
	str_len = (unsigned int)vsprintf ((char*)s, str, args);
	va_end(args);
	opencpu_uart_send(OPENCPU_MAIN_UART, s, str_len);
}

void custom_uart_init(void)
{
	opencpu_uart_open(OPENCPU_MAIN_UART, HAL_UART_BAUDRATE_115200, user_uart_callback);
}
	
