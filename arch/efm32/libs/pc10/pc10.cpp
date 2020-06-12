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

//#define	PC10_HALF_RANGE		8000.0			/* 800  KPa */
//#define	PC10_HALF_RANGE		175.0			/* 17.5 KPa */

//#define	D2429			1
#define	GENERAL1912			1
//#define	D15942			247
//#define	D15941			248

//#define	D2485			1004
//#define	D2477			1003
//#define	D2429			1002
//#define	D2421			1001

#ifdef GENERAL1912
#define PC10_HIGH		14744
#define PC10_MID		8191
#define PC10_LOW		1628
#endif

#ifdef D2429
#define PC10_HIGH		14716
#define PC10_MID		8197
#define PC10_LOW		1941
#endif

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

void pressure_init(int scl, int sda)
{
	wire_begin(scl, sda);
}
 
/*
 * Return value:
 *   18.00bar: Max valid value (16383)
 *   -0.15bar: Min valid value (1501)
 *   -1.0 bar: Bus error, no sensor connected, not wakeup
 *   -2.0 bar: Out of low range
 *   -3.0 bar: Out of high range
 *
 * Old Return value:
 *   78.00bar: Bus error
 *   18.00bar: Max valid value
 *   -1.0 bar: Out of low range value
*/
float get_pressure()
{
	uint16_t pv = 0;
	float p = 0.0;

	pc10_wakeup();

	i2c_delay(15*1000);		/* delay 11ms */

	pv = pc10_read();

	//Serial.print("pc10 = ");
	//Serial.println(pv, HEX);

	if (pv > 1500 && pv < PC10_MID) {

		p = PC10_HALF_RANGE / (PC10_MID - PC10_LOW) * (pv - PC10_LOW);

	} else if (pv >= PC10_MID && pv < 15000) {

		// If pv = 65535, then p = 78.006
		p = PC10_HALF_RANGE / (PC10_HIGH - PC10_MID) * (pv - PC10_MID) + PC10_HALF_RANGE;

	} else if (pv == 65535) {

		return -1.0;		// Bus error, no sensor connected, not wakeup ...

	} else if (pv <= 1500) {

		return -2.0;		// Out of low range

	} else if (pv >= 15000 && pv <= 0x3FFF) {

		return -3.0;		// Out of high range
	}

	// The unit of p is hPa(mbar)
	// p/1000.0 = bar (0.1MPa)
	p /= 1000.0;

	return p;
}

/*
 * Return value:
 *   18367.3469 cm: Max valid value (16383)
 *  -153.061224 cm: Min valid value (1501)
 *  -1.0 cm: Bus error, no sensor connected, not wakeup
 *  -2.0 cm: Out of low range
 *  -3.0 cm: Out of high range
*/
float get_water_h()
{
	uint16_t pv = 0;
	float p = 0.0;

	pc10_wakeup();

	i2c_delay(15*1000);		/* delay 11ms */

	pv = pc10_read();

	//Serial.print("pc10 = ");
	//Serial.println(pv, HEX);

	if (pv > 1500 && pv < PC10_MID) {

		p = PC10_HALF_RANGE / (PC10_MID - PC10_LOW) * (pv - PC10_LOW);

	} else if (pv >= PC10_MID && pv < 15000) {

		// If pv = 65535, then p = 78.006
		p = PC10_HALF_RANGE / (PC10_HIGH - PC10_MID) * (pv - PC10_MID) + PC10_HALF_RANGE;

	} else if (pv == 65535) {

		return -1.0;		// Bus error, no sensor connected, not wakeup ...

	} else if (pv <= 1500) {

		return -2.0;		// Out of low range

	} else if (pv >= 15000 && pv <= 0x3FFF) {

		return -3.0;		// Out of high range
	}

	// The unit of p is hPa(mbar)
	// p * 100 / (9.8*1000) * 100cm =  CM

	return (p * 1.02040816);
}

/*
 * Returned value:
 *   65535: Bus error
 *       0: The min value
 *   16338: The max value
*/
uint16_t pc10_read()
{
	uint8_t buf[LEN] = {};
	uint8_t i;

	wire_beginTransmission(PC10_ADDR);
	wire_write(0x09);
	wire_endTransmission();

	i2c_delay(2*1000);			/* delay 1.6ms */

	wire_requestFrom(PC10_ADDR, LEN);

	if (wire_available() != LEN) {
		// The returned press = 78.00 bar
		return 65535;
	}

	for (i = 0; i < LEN; i++) {
		buf[i] = wire_read();
	}
		
	// The max value is 0x3FFF = 16383
	// and the max press = 18bar
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
