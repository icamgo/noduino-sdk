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

//#define USE_MCP342x

#define PC10_ADDR		0x28
#define LEN				2

//#define	D15942			247
#define	D15941				248

#ifdef D15941
#define PC10_HIGH		14716
#define PC10_MID		8181
#define PC10_LOW		1640
#endif

#ifdef D15942
#define PC10_HIGH		14724
#define PC10_MID		8187
#define PC10_LOW		1642
#endif

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

/* Return hPa(mbar) */
float get_pressure()
{
#ifdef USE_MCP342x
	float ad = get_adc_uv();
	return ad;
#else
	uint16_t pv = 0;
	float p;

	pc10_wakeup();

	delay(15);		/* delay 11ms */

	pv = pc10_read();

//	Serial.print("pc10 = ");
//	Serial.println(pv, HEX);

	if (pv >= PC10_LOW && pv < PC10_MID) {
		p = 8000.0 / (PC10_MID - PC10_LOW) * (pv - PC10_LOW);
	} else if (pv >= PC10_MID && pv <= PC10_HIGH) {
		p = 8000.0 / (PC10_HIGH - PC10_MID) * (pv - PC10_MID) + 8000.0;
	} else {
		p = -1.0;
	}
	return p;
#endif
}

uint16_t pc10_read()
{
	uint8_t buf[LEN] = {};
	uint8_t i;

	Wire.beginTransmission(PC10_ADDR);
	Wire.write(0x09);
	Wire.endTransmission();

	delay(2);		/* delay 1.6ms */

	Wire.requestFrom(PC10_ADDR, LEN);

	if (Wire.available() != LEN)
		return -1;

	for (i = 0; i < LEN; i++) {
		buf[i] = Wire.read();
	}
	
	return ((buf[0] << 8) & 0x3FFF) | buf[1];
}

void pc10_wakeup()
{
	Wire.requestFrom(PC10_ADDR, 0);
}
