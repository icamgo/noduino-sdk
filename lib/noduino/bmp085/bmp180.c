/*
 *  Copyright (c) 2015 - 2025 MaiKe Labs
 *
 *  Library for BMP085 Digital pressure sensor 
 *
 *  This library is ported from adafruit Arduino BMP085 project
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

bool ICACHE_FLASH_ATTR bmp180_begin()
{
  uint8_t mode = BMP085_ULTRAHIGHRES;
  if (mode > BMP085_ULTRAHIGHRES) 
    mode = BMP085_ULTRAHIGHRES;
  bmp180_oversampling = mode;

  wire_begin();

  if (bmp180_read8(0xD0) != 0x55) return false;

  /* bmp180_read calibration data */
  ac1 = bmp180_read16(BMP085_CAL_AC1);
  ac2 = bmp180_read16(BMP085_CAL_AC2);
  ac3 = bmp180_read16(BMP085_CAL_AC3);
  ac4 = bmp180_read16(BMP085_CAL_AC4);
  ac5 = bmp180_read16(BMP085_CAL_AC5);
  ac6 = bmp180_read16(BMP085_CAL_AC6);

  b1 = bmp180_read16(BMP085_CAL_B1);
  b2 = bmp180_read16(BMP085_CAL_B2);

  mb = bmp180_read16(BMP085_CAL_MB);
  mc = bmp180_read16(BMP085_CAL_MC);
  md = bmp180_read16(BMP085_CAL_MD);
#if (BMP085_DEBUG == 1)
  serial_printf("ac1 = %d\r\n",ac1);
  serial_printf("ac2 = %d\r\n",ac2);
  serial_printf("ac3 = %d\r\n",ac3);
  serial_printf("ac4 = %d\r\n",ac4);
  serial_printf("ac5 = %d\r\n",ac5);
  serial_printf("ac6 = %d\r\n",ac6);

  serial_printf("b1 = %d\r\n",b1);
  serial_printf("b2 = %d\r\n",b2);

  serial_printf("mb = %d\r\n",mb);
  serial_printf("mc = %d\r\n",mc);
  serial_printf("md = %d\r\n",md);
#endif

  return true;
}

int32_t ICACHE_FLASH_ATTR bmp180_computeB5(int32_t UT)
{
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
  return X1 + X2;
}

uint16_t ICACHE_FLASH_ATTR bmp180_readRawTemperature()
{
  bmp180_write8(BMP085_CONTROL, BMP085_READTEMPCMD);
  delay(5);
#if BMP085_DEBUG == 1
  serial_printf("Raw temp: %d\r\n", bmp180_read16(BMP085_TEMPDATA));
#endif
  return bmp180_read16(BMP085_TEMPDATA);
}

uint32_t ICACHE_FLASH_ATTR bmp180_readRawPressure()
{
  uint32_t raw;

  bmp180_write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (bmp180_oversampling << 6));

  if (bmp180_oversampling == BMP085_ULTRALOWPOWER) 
    delay(5);
  else if (bmp180_oversampling == BMP085_STANDARD) 
    delay(8);
  else if (bmp180_oversampling == BMP085_HIGHRES) 
    delay(14);
  else 
    delay(26);

  raw = bmp180_read16(BMP085_PRESSUREDATA);

  raw <<= 8;
  raw |= bmp180_read8(BMP085_PRESSUREDATA+2);
  raw >>= (8 - bmp180_oversampling);

 /* this pull broke stuff, look at it later?
  if (bmp180_oversampling==0) {
    raw <<= 8;
    raw |= bmp180_read8(BMP085_PRESSUREDATA+2);
    raw >>= (8 - bmp180_oversampling);
  }
 */

#if BMP085_DEBUG == 1
  serial_printf("Raw pressure: %d\r\n", raw);
#endif
  return raw;
}


int32_t ICACHE_FLASH_ATTR bmp180_readPressure()
{
  int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
  uint32_t B4, B7;

  UT = bmp180_readRawTemperature();
  UP = bmp180_readRawPressure();

#if BMP085_DEBUG == 1
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

#if BMP085_DEBUG == 1
  serial_printf("X1 = %d\r\n", X1);
  serial_printf("X2 = %d\r\n", X2);
  serial_printf("B5 = %d\r\n", B5);
#endif

  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1*4 + X3) << bmp180_oversampling) + 2) / 4;

#if BMP085_DEBUG == 1
  serial_printf("B6 = %d\r\n", B6);
  serial_printf("X1 = %d\r\n", X1);
  serial_printf("X2 = %d\r\n", X2);
  serial_printf("B3 = %d\r\n", B3);
#endif

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> bmp180_oversampling );

#if BMP085_DEBUG == 1
  serial_printf("X1 = %d\r\n", X1);
  serial_printf("X2 = %d\r\n", X2);
  serial_printf("B4 = %d\r\n", B4);
  serial_printf("B7 = %d\r\n", B7);
#endif

  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

#if BMP085_DEBUG == 1
  serial_printf("p = %d\r\n", p);
  serial_printf("X1 = %d\r\n", X1);
  serial_printf("X2 = %d\r\n", X2);
#endif

  p = p + ((X1 + X2 + (int32_t)3791)>>4);
#if BMP085_DEBUG == 1
  serial_printf("p = %d\r\n", p); 
#endif
  return p;
}

int32_t ICACHE_FLASH_ATTR bmp180_readSealevelPressure(float altitude_meters)
{
  float pressure = bmp180_readPressure();
  return (int32_t)(pressure / pow(1.0-altitude_meters/44330, 5.255));
}

float ICACHE_FLASH_ATTR bmp180_readTemperature()
{
  int32_t UT, B5;     // following ds convention
  float temp = 0;

  UT = bmp180_readRawTemperature();

#if BMP085_DEBUG == 1
  serial_printf("readTemp: UT: %d\r\n", UT);
  // use datasheet numbers!
  UT = 27898;
  ac6 = 23153;
  ac5 = 32757;
  mc = -8711;
  md = 2868;
#endif

  B5 = bmp180_computeB5(UT);
  temp = (((float)B5 + 8) / 16) / 10;

  return temp;
}

float ICACHE_FLASH_ATTR bmp180_readAltitude(float sealevelPressure)
{
  float altitude;

  float pressure = bmp180_readPressure();

  altitude = 44330 * (1.0 - pow(pressure /sealevelPressure,0.1903));

  return altitude;
}

uint8_t ICACHE_FLASH_ATTR bmp180_read8(uint8_t a)
{
  uint8_t ret;

  wire_beginTransmission(BMP085_I2CADDR);	// start transmission to device 
  wire_write(a);							// sends register address to read from
  wire_endTransmission();					// end transmission
  
  wire_beginTransmission(BMP085_I2CADDR);	// start transmission to device 
  wire_requestFrom(BMP085_I2CADDR, 1);		// send data n-bytes read
  ret = wire_read();						// receive DATA
  wire_endTransmission();					// end transmission

  return ret;
}

uint16_t ICACHE_FLASH_ATTR bmp180_read16(uint8_t a)
{
  uint16_t ret;

  wire_beginTransmission(BMP085_I2CADDR);	// start transmission to device 
  wire_write(a);							// sends register address to read from
  wire_endTransmission();					// end transmission
  
  wire_beginTransmission(BMP085_I2CADDR);	// start transmission to device 
  wire_requestFrom(BMP085_I2CADDR, 2);		// send data n-bytes read
  ret = wire_read();						// receive DATA
  ret <<= 8;
  ret |= wire_read();						// receive DATA
  wire_endTransmission();					// end transmission

  return ret;
}

void ICACHE_FLASH_ATTR bmp180_write8(uint8_t a, uint8_t d)
{
  wire_beginTransmission(BMP085_I2CADDR);	// start transmission to device 
  wire_write(a);							// sends register address to read from
  wire_write(d);							// write data
  wire_endTransmission();					// end transmission
}
