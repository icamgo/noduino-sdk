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
#include "noduino.h"
#include "softuart.h"

Softuart softuart;

void setup()
{
	serial_begin(115200);
	delay(100);

	Softuart_SetPinRx(&softuart,14);	
	Softuart_SetPinTx(&softuart,12);
	Softuart_Init(&softuart,9600);
	//set pin 13 as output to control tx enable/disable of rs485
	pinMode(13, OUTPUT);
	Softuart_EnableRs485(&softuart, 13);

	serial_printf("RS485 test...\r\n");
}

void loop()
{
	uint8_t c = 0;
	char buf[128] = { 0 };

	if (serial_available()) {
		c = serial_read();
		serial_printf("uart rx: %c, send via rs485...\r\n", c);	
		Softuart_Putchar(&softuart, c);
	}

	if(Softuart_Available(&softuart)) {
		Softuart_Readline(&softuart, buf, 128);
		serial_printf("RS485 rx: [%s]\r\n", buf);
	}

	delay(100);
}
