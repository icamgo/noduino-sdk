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

#ifdef CONFIG_SOFTI2C
#include "twi.h"

#define I2C_SDA					12		/* PIN23_PE12_D12 */
#define I2C_SCL					13		/* PIN24_PE13_D13 */

#else
#include "i2c.h"
#endif

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
	delay(10);

	digitalWrite(RESET_PIN, LOW);
	delayMicroseconds(200);
	digitalWrite(RESET_PIN, HIGH);
	delayMicroseconds(800);

#ifdef CONFIG_SOFTI2C
	wire_begin(I2C_SCL, I2C_SDA);
#else
	i2c_init();
#endif
}

void loop()
{
	int error, address;
	int n;
	uint8_t txbuf[4], rxbuf[4];

	txbuf[0] = 1;

	Serial.println("I2C Scanning...");

	n = 0;
	for (address = 0; address <= 127; address++) {

#ifdef CONFIG_SOFTI2C
		wire_beginTransmission(address);
		error = wire_endTransmission();
#else
		error = i2c_write(address, txbuf, 0, rxbuf, 0);
#endif

		if (error == 0) {
			Serial.print("I2C device found at address 0x");
			Serial.println(address, HEX);
			n++;
		} else {
			Serial.print("Unknow error at address: ");
			Serial.println(error, DEC);
		}
	}
	if (n == 0)
		Serial.println("No I2C devices found");
	else
		Serial.println("done");

	delay(25*1000);
}
