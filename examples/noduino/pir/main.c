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

static bool st = 0;

void do_pir()
{
	st = digitalRead(D5);
	serial_printf("pir =%d\r\n", st);
}

void setup()
{
	serial_begin(115200);

	pinMode(D5, INPUT);

	attachInterrupt(D5, do_pir, CHANGE);

	serial_printf("%s\r\n", "Testing the pir");
}

void loop()
{
}
