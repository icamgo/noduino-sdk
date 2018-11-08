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

void loop()
{
	byte error, address;
	int nDevices;

	serial_printf("I2C Scanning...\r\n");

	nDevices = 0;
	for (address = 0; address <= 127; address++) {
		// The i2c_scanner uses the return value of
		// the wire_endTransmisstion to see if
		// a device did acknowledge to the address.
		wire_beginTransmission(address);
		error = wire_endTransmission();

		if (error == 0) {
			serial_printf("I2C device found at address 0x%02X\r\n", address);
			nDevices++;
		} else if (error == 4) {
			serial_printf("Unknow error at address 0x%02X\r\n", address);
		}
	}
	if (nDevices == 0)
		serial_printf("No I2C devices found\r\n");
	else
		serial_printf("done\r\n");

	delay(25*1000);
}
