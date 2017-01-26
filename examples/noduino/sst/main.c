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
#include "noduino.h"

void setup()
{
	pinMode(D4, OUTPUT);
	pinMode(D5, OUTPUT);
	pinMode(D6, OUTPUT);
	digitalWrite(D4, LOW);
	digitalWrite(D5, LOW);
	digitalWrite(D6, LOW);
}

void loop()
{
	digitalWrite(D4, HIGH);	// light on
	delay(1000);				// delay 1000ms
	digitalWrite(D4, LOW);		// light off
	delay(1000);				// delay 1000ms
	digitalWrite(D5, HIGH);	// light on
	delay(2000);				// delay 1000ms
	digitalWrite(D5, LOW);		// light off
	delay(1000);				// delay 1000ms
	digitalWrite(D6, HIGH);	// light on
	delay(2000);				// delay 1000ms
	digitalWrite(D6, LOW);		// light off
	delay(1000);				// delay 1000ms
}
