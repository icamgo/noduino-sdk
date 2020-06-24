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

#include "sh1107.h"

#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8  */
#define RESET_PIN				16		/* PIN21_PF02_D16 */

void setup()
{
//	Serial.setRouteLoc(1);
//	Serial.begin(115200);

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
	i2c_init(0x78);
	//i2c_init(0x3c);
#endif

    sh1107_oled.init(SH1107G);							// initialize SEEED OLED display
    sh1107_oled.clearDisplay();							// Clear Display.
    sh1107_oled.setNormalDisplay();						// Set Normal Display Mode
    sh1107_oled.setVerticalMode();						// Set to vertical mode for displaying text

    for (char i = 0; i < 16 ; i++) {
        sh1107_oled.setTextXY(i, 0);					// set Cursor to ith line, 0th column
        sh1107_oled.setGrayLevel(i);					// Set Grayscale level. Any number between 0 - 15.
        sh1107_oled.putString("Hello World OLED");		// Print Hello World
    }
}

void loop()
{

}
