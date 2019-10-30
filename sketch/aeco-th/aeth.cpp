/*
 *  Copyright (c) 2015 - 2025 MaiKe Labs
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

int LED = 2;

void setup()
{
	pinMode(LED, OUTPUT);		// set gpio2 to output mode
	digitalWrite(LED, LOW);
}

void loop()
{
	digitalWrite(LED, HIGH);	// light on
	delay(3000);			// delay 1000ms
	digitalWrite(LED, LOW);		// light off
	delay(3000);			// delay 1000ms
}
