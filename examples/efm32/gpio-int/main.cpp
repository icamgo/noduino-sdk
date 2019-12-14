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

int c = -1;

void handle_d0_int()
{
	c = 0;
	// D0(PA0/P1) int
}

void handle_d11_int()
{
	// PT02
	c = 11;
	// D11(PD7/P14) int
}

void handle_d16_int()
{
	// RES
	c = 16;
	// D16(PF02/P21) int
}

void handle_d12_int()
{
	// SDA
	c = 12;
	// D12(PE12/P23) int
}

void handle_d13_int()
{
	// SCL
	c = 13;
	// D13(PE13/P24) int
}

void setup()
{
	Serial.setRouteLoc(1);
	Serial.begin(115200);

	pinMode(0, INPUT);
	attachInterrupt(0, handle_d0_int, FALLING);

	pinMode(11, INPUT);
	attachInterrupt(11, handle_d11_int, FALLING);

	pinMode(16, INPUT);
	attachInterrupt(16, handle_d16_int, FALLING);

	pinMode(12, INPUT);
	attachInterrupt(12, handle_d12_int, FALLING);

	pinMode(13, INPUT);
	attachInterrupt(13, handle_d13_int, FALLING);

	Serial.println("Interrupt testing start...");
}

void loop()
{
	Serial.print("Main loooop, c = ");
	Serial.println(c);

	delay(2000);
}
