/*
 *  Copyright (c) 2015 - 2025 MaiKe Labs
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
#include "softi2c.h"
#include "pcf8563.h"

#include "rtcdriver.h"
#include "em_emu.h"

#define SDA_PIN					12		/* PIN23_PE12 */
#define SCL_PIN					13		/* PIN24_PE13 */

#define	INT_PIN					16		/* PIN21_PF02_D16 */

void check_rtc()
{
	pcf8563_reset_timer();

	Serial.print("check ctrl2: ");
	Serial.println(pcf8563_get_ctrl2(), HEX);
}

void setup()
{
	/* Initialize EM23 with default parameters */
	//EMU_EM23Init_TypeDef em23Init = EMU_EM23INIT_DEFAULT;
	//EMU_EM23Init(&em23Init);
	//CMU_OscillatorEnable(cmuOsc_LFXO, false, true);
	//RTCDRV_Init();

	pinMode(INT_PIN, INPUT);
	attachInterrupt(INT_PIN, check_rtc, FALLING);

	Serial.setRouteLoc(1);
	Serial.begin(115200);

	pcf8563_init(SCL_PIN, SDA_PIN);
	pcf8563_set_from_int(2020, 3, 19, 17, 21, 25);

	Serial.println("RTC testing start...");

	pcf8563_set_timer(1);

	Serial.print("CTRL2: ");
	Serial.println(pcf8563_get_ctrl2(), HEX);
}

void loop()
{
	Serial.print("Now = ");
	Serial.println(pcf8563_now());

	delay(5000);

	EMU_EnterEM2(true);
}
