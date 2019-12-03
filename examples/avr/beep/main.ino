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
#include <Arduino.h>

// 10 ms, [20, 100]
uint32_t interval = 20;

void change_interval()
{
	interval += 10;
	Serial.print("Beep: ");
	Serial.println(interval);
}

void setup()
{
	// beep
	pinMode(7, OUTPUT);
	digitalWrite(7, LOW);
	
	// key
	pinMode(2, INPUT_PULLUP);

	attachInterrupt(0, change_interval, FALLING);

	Serial.begin(115200);
}

void beep(int c)
{
	switch (c) {
		case 3:
			digitalWrite(7, HIGH);
			delay(interval);
			digitalWrite(7, LOW);
			delay(40);
		case 2:
			digitalWrite(7, HIGH);
			delay(interval);
			digitalWrite(7, LOW);
			delay(40);
		case 1:
			digitalWrite(7, HIGH);
			delay(interval);
			digitalWrite(7, LOW);
	}
}

void loop()
{
	Serial.print("Beep intervalval: ");
	Serial.println(interval);

	beep(1);
	delay(1500);
	beep(2);
	delay(1500);
	beep(3);
	delay(3500);
}
