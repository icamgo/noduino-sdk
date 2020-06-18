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

#include "Arduino.h"
#include "twi.h"

#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define RESET_PIN				16

void setup()
{
	Serial.setRouteLoc(1);
	Serial.begin(115200);

	pinMode(PWR_CTRL_PIN, OUTPUT);
	pinMode(RESET_PIN, OUTPUT);

	// power on sh1107 oled
	digitalWrite(PWR_CTRL_PIN, HIGH);

	digitalWrite(RESET_PIN, LOW);
	delayMicroseconds(100);
	digitalWrite(RESET_PIN, HIGH);
	delayMicroseconds(500);

	wire_begin(13, 12);
}

void loop()
{
	byte error, address;
	int n;

	Serial.println("I2C Scanning...");

	n = 0;
	for (address = 0; address <= 127; address++) {

		wire_beginTransmission(address);
		error = wire_endTransmission();

		if (error == 0) {
			Serial.print("I2C device found at address 0x");
			Serial.println(address, HEX);
			n++;
		} else if (error == 4) {
			Serial.print("Unknow error at address: ");
			Serial.println(address, HEX);
		}
	}
	if (n == 0)
		Serial.println("No I2C devices found");
	else
		Serial.println("done");

	delay(25*1000);
}
