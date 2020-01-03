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
#include "sts.h"

SHTSensor sts;

void setup()
{
	Serial.setRouteLoc(1);
	Serial.begin(115200);

	sts.init();

	Serial.println("STS testing start...");
}

void loop()
{
	float temp = 0.0, humi = 0.0;

	sts.readSample();
	
	Serial.print("Temperatur = ");
	Serial.print(sts.getTemperature(), 2);
	Serial.println(" C");

	delay(3000);
}
