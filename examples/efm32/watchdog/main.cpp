/*
 *  Copyright (c) 2019 - 2029 MaiKe Labs
 *
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

#include "em_wdog.h"


void setup()
{
	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = wdogPeriod_8k;	/* 4k 1kHz periods should give ~4 seconds */

	// wdogPeriod_256k ---> 262145 clock periods, 256s

	Serial.setRouteLoc(1);
	Serial.begin(115200);

	/* Start watchdog */
	WDOG_Init(&wInit);
}

void loop()
{
	Serial.print("Read internal temperature :");

	float temp = adc.temperatureCelsius();
	Serial.print(temp);
	Serial.println(" 'C");

	delay(2000);

	Serial.println("Haha, delay 2s....");

	EMU_EnterEM2(true);
}
