/*
 *  Copyright (c) 2018 - 2028 MaiKe Labs
 *
 *  Library for SHT3x digital temperature sensor
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

#ifndef __SHT3X_H__
#define __SHT3X_H__

#include <Arduino.h>

//#define	DEBUG				1

#ifdef DEBUG
#define	INFO(x)				Serial.println(x)
#else
#define	INFO(x)
#endif

#define	SHT3X_ADDR					SHT3X_ADDR_LOW
#define	SHT3X_ADDR_LOW				0x44
#define	SHT3X_ADDR_HIGH				0x45

#define CLEAR_STATUS				0x3041
#define SOFT_RESET					0x30A2

#define READ_HIGH_RES				0x2C06
#define READ_MEDIUM_RES				0x2C0D
#define READ_LOW_RES				0x2C0F

#define R_HIGH_ALERT_SET			0xE11F
#define R_HIGH_ALERT_CLR			0xE114
#define R_LOW_ALERT_SET				0xE102
#define R_LOW_ALERT_CLR				0xE109

#define W_HIGH_ALERT_SET			0x611D
#define W_HIGH_ALERT_CLR			0x6116
#define W_LOW_ALERT_SET				0x610B
#define W_LOW_ALERT_CLR				0x6100

int sht3x_read_sensor(float *t, float *h);
float sht3x_get_temp(void);
float sht3x_get_humi(void);
void sht3x_show_alert();

uint8_t sht3x_reset(void);
uint8_t sht3x_init(uint8_t scl, uint8_t sda);
#endif
