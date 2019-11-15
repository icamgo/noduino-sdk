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
#include "em_cmu.h"
#include "em_emu.h"

#include "tempdrv.h"

void tempdrv_callback(int8_t temp, TEMPDRV_LimitType_t limit)
{
	Serial.println("Temp callback is trigged!");
}

void setup()
{
	Serial.setRouteLoc(1);
	Serial.begin(115200);

	TEMPDRV_Init();

	// Register a callback at 10 degrees above current temperature
	TEMPDRV_RegisterCallback(TEMPDRV_GetTemp()+1, TEMPDRV_LIMIT_HIGH, tempdrv_callback);
}

void loop()
{
	uint8_t temp = 0;

	temp = TEMPDRV_GetTemp();

	Serial.print("The temperature of EMU: ");
	Serial.println(temp);

	delay(5000);

	//EMU_EnterEM2(true);
}
