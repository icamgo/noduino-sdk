/*
 *  Copyright (c) 2015 - 2025 MaiKe Labs
 *
 *  Connect SDA to i2c data  - GPIO4
 *  Connect SCL to i2c clock - GPIO5
 *
 *  Scanning addresses changed from 1..127
 *  This sketch tests the standard 7-bit addresses
 *  Devices with higher bit address might not be seen properly.
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

void setup()
{
	serial_begin(115200);
	wire_begin();
}

int read_bytes(uint8_t *val, int len)
{
	int error, x;
	uint8_t cmd, addr = 0x30;

	cmd = val[0];

	wire_beginTransmission(addr);
	wire_write(cmd);

	error = wire_endTransmission();

	if (error == 0) {

		wire_requestFrom(addr, len);

		while (wire_available() != len);

		for (x = 0; x < len; x++) {
			val[x] = wire_read();
		}

		return (1);
	}
	return (0);
}


void loop()
{
	uint8_t error;

	uint8_t data[16], len = 12;

	int i = 0;

	serial_printf("Reading i2c device...\r\n");

	memset(data, 0, len);

	data[0] = 0x34;	// set the i2c cmd

	if (read_bytes(data, len) == 1) {

		serial_printf("bytes: ");

		for (i = 0; i < len; i++ )
			serial_printf("%02X ", data[i]);

		serial_printf("\r\n");

	} else {
		serial_printf("Read bytes failed\r\n");	
	}
	
	memset(data, 0, len);

	data[0] = 0x46;	// set the i2c cmd

	if (read_bytes(data, len) == 1) {

		serial_printf("bytes: ");

		for (i = 0; i < len; i++ )
			serial_printf("%02X ", data[i]);

		serial_printf("\r\n");

	} else {
		serial_printf("Read bytes failed\r\n");	
	}

	delay(30*1000);
}
