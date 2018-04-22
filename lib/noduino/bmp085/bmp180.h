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

#ifndef __BMP180_H__
#define __BMP180_H__

#include "noduino.h"

#define BMP180_DEBUG	0

#define BMP180_I2CADDR	0x77

#define BMP180_ULTRALOWPOWER 0
#define BMP180_STANDARD      1
#define BMP180_HIGHRES       2
#define BMP180_ULTRAHIGHRES  3
#define BMP180_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP180_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP180_CAL_AC3           0xAE  // R   Calibration data (16 bits)    
#define BMP180_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP180_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP180_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP180_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP180_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP180_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP180_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP180_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP180_CONTROL           0xF4 
#define BMP180_TEMPDATA          0xF6
#define BMP180_PRESSUREDATA      0xF6
#define BMP180_READTEMPCMD       0x2E
#define BMP180_READPRESSURECMD   0x34


bool bmp180_begin();  // by default go highres
float bmp180_readTemperature();
float bmp180_readAltitude(float sealevelPressure); // std atmosphere
int32_t bmp180_readPressure();
int32_t bmp180_readSealevelPressure(float altitude_meters);
uint16_t bmp180_readRawTemperature();
uint32_t bmp180_readRawPressure();
  
int32_t bmp180_computeB5(int32_t UT);
uint8_t bmp180_read8(uint8_t addr);
uint16_t bmp180_read16(uint8_t addr);
void bmp180_write8(uint8_t addr, uint8_t data);

uint8_t bmp180_oversampling;
#endif
