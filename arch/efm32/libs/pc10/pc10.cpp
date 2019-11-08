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

#include "pc10.h"
#include "softi2c.h"

#define PC10_ADDR		0x28
#define LEN				2

//#define	D15942			247
//#define	D15941				248
#define	D15941				101

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

void pressure_init()
{
	wire_begin();
}

/* Return hPa(mbar) */
float get_pressure()
{
	uint16_t pv = 0;
	float p;

	pc10_wakeup();

	delayMicroseconds(350);

	pv = pc10_read();

	Serial.print("pc10 = ");
	Serial.println(pv, HEX);

	if (pv >= PC10_LOW && pv < PC10_MID) {
		p = 8000.0 / (PC10_MID - PC10_LOW) * (pv - PC10_LOW);
	} else if (pv >= PC10_MID && pv <= PC10_HIGH) {
		p = 8000.0 / (PC10_HIGH - PC10_MID) * (pv - PC10_MID) + 8000.0;
	} else {
		p = -1.0;
	}
	return p;
}

uint16_t pc10_read()
{
	uint8_t buf[LEN] = {};
	uint8_t i;

	wire_beginTransmission(PC10_ADDR);
	wire_write(0x09);
	wire_endTransmission();

	delay(1);

	wire_requestFrom(PC10_ADDR, LEN);

	if (wire_available() != LEN)
		return -1;

	for (i = 0; i < LEN; i++) {
		buf[i] = wire_read();
	}
	
	return ((buf[0] << 8) & 0x3FFF) | buf[1];
}

void pc10_wakeup()
{
#if 1
	wire_requestFrom(PC10_ADDR, 1);
#else
	i2c_start();
	i2c_write_byte(0x51);
	i2c_stop();
#endif
}
