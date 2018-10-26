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

int write8(uint8_t addr, uint8_t d)
{
	int error = 0;

	wire_beginTransmission(addr);
	wire_write(d);

	error = wire_endTransmission();

	return error;
}

void loop()
{
	byte error, cmd;
	int n;

	byte addr = 0x10;

	serial_printf("I2C addr: 0x%02X write probing...\r\n", addr);

	for (cmd = 0; cmd <= 127; cmd++) {

		wire_beginTransmission(addr);

		wire_write(cmd);

		error = wire_endTransmission();

		if (error == 0) {
			serial_printf("I2C write cmd 0x%02X is OK\r\n", cmd);
		} else {
			serial_printf("Unknow error when write cmd 0x%02X\r\n", cmd);
		}

		delay(500);
	}

	delay(25*1000);
}
