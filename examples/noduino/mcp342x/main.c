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
#include "mcp342x.h"

void setup()
{
	serial_begin(115200);
	wifi_set_opmode(NULL_MODE);

	mcp342x_init();
	mcp342x_set_oneshot();
}
 
void loop()
{
  serial_printf("Testing mcp3421 ADC...\r\n");
 
  serial_printf("uV = %d\r\n", mcp342x_get_uv());
 
  delay(5000);
}
