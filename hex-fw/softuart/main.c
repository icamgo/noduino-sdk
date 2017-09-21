/*
 * Copyright (c) 2016, Łukasz Marcin Podkalicki <lpodkalicki@gmail.com>
 * ATtiny13/008
 * Example of Software UART.
 */

#include "softuart.h"

int main(void)
{
	char *p, buff[16];

	uart_init(9600);

	//_delay_ms(2000);
	//uart_puts("Hello World!\n");

	/* loop */
	while (1) {
#if 0
		p = buff;
		while((*(p++) = uart_getc()) != '\n' && (p - buff) < 16);
		*p = 0;
		_delay_ms(10);
		uart_puts(buff);
#else
	_delay_ms(2000);
	uart_puts("Hello World!\r\n");
#endif
	}
}
