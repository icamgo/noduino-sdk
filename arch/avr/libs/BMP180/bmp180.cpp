/*
 *  Copyright (c) 2015 - 2025 MaiKe Labs
 *
 *  Library for BMP180 Digital pressure sensor 
 *
 *  This library is ported from adafruit Arduino BMP180 project
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

#include "bmp180.h"

int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;

uint8_t bmp180_oversampling;

bool bmp180_begin()
{
	uint8_t mode = BMP180_STANDARD;

	if (mode > BMP180_ULTRAHIGHRES)
		mode = BMP180_ULTRAHIGHRES;

	bmp180_oversampling = mode;

	INFOLN("BMP180 begin");

	bmp180_reset();
	delay(20);

	if (bmp180_read8(0xD0) != 0x55)
		return false;

	/* bmp180_read calibration data */
	ac1 = bmp180_read16(BMP180_CAL_AC1);
	ac2 = bmp180_read16(BMP180_CAL_AC2);
	ac3 = bmp180_read16(BMP180_CAL_AC3);
	ac4 = bmp180_read16(BMP180_CAL_AC4);
	ac5 = bmp180_read16(BMP180_CAL_AC5);
	ac6 = bmp180_read16(BMP180_CAL_AC6);

	b1 = bmp180_read16(BMP180_CAL_B1);
	b2 = bmp180_read16(BMP180_CAL_B2);

	mb = bmp180_read16(BMP180_CAL_MB);
	mc = bmp180_read16(BMP180_CAL_MC);
	md = bmp180_read16(BMP180_CAL_MD);

#ifdef BMP180_DEBUG
	INFO("ac1 = ");
	INFOLN(ac1);
	INFO("ac2 = ");
	INFOLN(ac2);
	INFO("ac3 = ");
	INFOLN(ac3);
	INFO("ac4 = ");
	INFOLN(ac4);
	INFO("ac5 = ");
	INFOLN(ac5);
	INFO("ac6 = ");
	INFOLN(ac6);

	INFO("b1 = ");
	INFOLN(b1);
	INFO("b2 = ");
	INFOLN(b2);

	INFO("mb = ");
	INFOLN(mb);
	INFO("mc = ");
	INFOLN(mc);
	INFO("md = ");
	INFOLN(md);
#endif

	return true;
}

void bmp180_reset()
{
	bmp180_write8(0xE0, 0xB6);
}

int32_t bmp180_computeB5(int32_t UT)
{
	int32_t X1 = (UT - (int32_t) ac6) * ((int32_t) ac5) >> 15;
	int32_t X2 = ((int32_t) mc << 11) / (X1 + (int32_t) md);
	return X1 + X2;
}

uint16_t bmp180_read_raw_temp()
{
	bmp180_write8(BMP180_CONTROL, BMP180_READTEMPCMD);
	delay(5);
#if BMP180_DEBUG == 1
	INFO("Raw temp: ");
	INFOLN(bmp180_read16(BMP180_TEMPDATA));
#endif
	return bmp180_read16(BMP180_TEMPDATA);
}

uint32_t bmp180_read_raw_press()
{
	uint32_t raw;

	bmp180_write8(BMP180_CONTROL,
		      BMP180_READPRESSURECMD + (bmp180_oversampling << 6));

	if (bmp180_oversampling == BMP180_ULTRALOWPOWER)
		delay(5);
	else if (bmp180_oversampling == BMP180_STANDARD)
		delay(8);
	else if (bmp180_oversampling == BMP180_HIGHRES)
		delay(14);
	else
		delay(26);

	raw = bmp180_read16(BMP180_PRESSUREDATA);

	raw <<= 8;
	raw |= bmp180_read8(BMP180_PRESSUREDATA + 2);
	raw >>= (8 - bmp180_oversampling);

	/* this pull broke stuff, look at it later?
	   if (bmp180_oversampling==0) {
	   raw <<= 8;
	   raw |= bmp180_read8(BMP180_PRESSUREDATA+2);
	   raw >>= (8 - bmp180_oversampling);
	   }
	 */

#if BMP180_DEBUG == 1
	INFO("Raw pressure: ");
	INFOLN(raw);
#endif
	return raw;
}

int32_t bmp180_read_press()
{
	int32_t B3, B5, B6, X1, X2, X3, p;
	uint32_t B4, B7;
	int32_t UT, UP;

	UT = bmp180_read_raw_temp();
	UP = bmp180_read_raw_press();

#if BMP180_DEBUG == 1
	// use datasheet numbers!
	UT = 27898;
	UP = 23843;
	ac6 = 23153;
	ac5 = 32757;
	mc = -8711;
	md = 2868;
	b1 = 6190;
	b2 = 4;
	ac3 = -14383;
	ac2 = -72;
	ac1 = 408;
	ac4 = 32741;
	bmp180_oversampling = 0;
#endif

	B5 = bmp180_computeB5(UT);

#if BMP180_DEBUG == 1
	INFO("X1 = ");
	INFOLN(X1);
	INFO("X2 = ");
	INFOLN(X2);
	INFO("B5 = ");
	INFOLN(B5);
#endif

	// do pressure calcs
	B6 = B5 - 4000;
	X1 = ((int32_t) b2 * ((B6 * B6) >> 12)) >> 11;
	X2 = ((int32_t) ac2 * B6) >> 11;
	X3 = X1 + X2;
	B3 = ((((int32_t) ac1 * 4 + X3) << bmp180_oversampling) + 2) / 4;

#if BMP180_DEBUG == 1
	INFO("B6 = ");
	INFOLN(B6);
	INFO("X1 = ");
	INFOLN(X1);
	INFO("X2 = ");
	INFOLN(X2);
	INFO("B3 = ");
	INFOLN(B3);
#endif

	X1 = ((int32_t) ac3 * B6) >> 13;
	X2 = ((int32_t) b1 * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = ((uint32_t) ac4 * (uint32_t) (X3 + 32768)) >> 15;
	B7 = ((uint32_t) UP - B3) * (uint32_t) (50000UL >> bmp180_oversampling);

#if BMP180_DEBUG == 1
	INFO("X1 = ");
	INFOLN(X1);
	INFO("X2 = ");
	INFOLN(X2);
	INFO("B4 = ");
	INFOLN(B4);
	INFO("B7 = ");
	INFOLN(B7);
#endif

	if (B7 < 0x80000000) {
		p = (B7 * 2) / B4;
	} else {
		p = (B7 / B4) * 2;
	}
	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;

#if BMP180_DEBUG == 1
	INFO("p = ");
	INFOLN(p);
	INFO("X1 = ");
	INFOLN(X1);
	INFO("X2 = ");
	INFOLN(X2);
#endif

	p = p + ((X1 + X2 + (int32_t) 3791) >> 4);
#if BMP180_DEBUG == 1
	INFO("p = ");
	INFOLN(p);
#endif
	return p;
}

int32_t bmp180_read_sea_level_press(float altitude_meters)
{
	float pressure = bmp180_read_press();
	return (int32_t) (pressure / pow(1.0 - altitude_meters / 44330, 5.255));
}

float bmp180_read_temp()
{
	int32_t B5;		// following ds convention
	uint32_t UT;
	float temp = 0;

	UT = bmp180_read_raw_temp();

#if BMP180_DEBUG == 1
	INFO("readTemp: UT: ");
	INFOLN(UT);
	// use datasheet numbers!
	UT = 27898;
	ac6 = 23153;
	ac5 = 32757;
	mc = -8711;
	md = 2868;
#endif

	B5 = bmp180_computeB5(UT);
	temp = (B5 + 8) >> 4;
	temp /= 10.0;

	return temp;
}

float bmp180_read_alt(float sealevelPressure)
{
	float altitude;

	float pressure = bmp180_read_press();

	altitude = 44330 * (1.0 - pow(pressure / sealevelPressure, 0.1903));

	return altitude;
}

uint8_t bmp180_read8(uint8_t a)
{
	uint8_t ret;

	Wire.beginTransmission(BMP180_I2CADDR);	// start transmission to device 
	Wire.write(a);		// sends register address to read from
	Wire.endTransmission();	// end transmission

	Wire.beginTransmission(BMP180_I2CADDR);	// start transmission to device 
	Wire.requestFrom(BMP180_I2CADDR, 1);	// send data n-bytes read
	ret = Wire.read();	// receive DATA
	Wire.endTransmission();	// end transmission

	return ret;
}

uint16_t bmp180_read16(uint8_t a)
{
	uint16_t ret;

	Wire.beginTransmission(BMP180_I2CADDR);	// start transmission to device 
	Wire.write(a);		// sends register address to read from
	Wire.endTransmission();	// end transmission

	Wire.beginTransmission(BMP180_I2CADDR);	// start transmission to device 
	Wire.requestFrom(BMP180_I2CADDR, 2);	// send data n-bytes read
	ret = Wire.read();	// receive DATA
	ret <<= 8;
	ret |= Wire.read();	// receive DATA
	Wire.endTransmission();	// end transmission

	return ret;
}

void bmp180_write8(uint8_t a, uint8_t d)
{
	Wire.beginTransmission(BMP180_I2CADDR);	// start transmission to device 
	Wire.write(a);		// sends register address to read from
	Wire.write(d);		// write data
	Wire.endTransmission();	// end transmission
}
