/*
 *  Copyright (c) 2019 - 2029 MaiKe Labs
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

#include "pressure.h"

void pc10_init()
{
	Wire.begin();
}

void pressure_init()
{
#ifdef USE_MCP342x
	adc_init();
#else
	pc10_init();
#endif
}

float get_pressure()
{
#ifdef USE_MCP342x
	float ad = get_adc_uv();
	return ad;
#else

#endif
}

#define ADDR	0x28
#define LEN		2
uint16_t pc10_read()
{
	uint8_t buf[LEN] = {};
	uint8_t i;

	Wire.requestFrom(ADDR, LEN);

	if (Wire.available() != LEN)
		return -1;

	for (i = 0; i < LEN; i++) {
		buf[i] = Wire.read();
	}
	
	return ((buf[0] << 8) & 0x3FFF) | buf[1];
}
