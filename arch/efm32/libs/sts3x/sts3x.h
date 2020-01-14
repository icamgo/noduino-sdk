/*
 *  Copyright (c) 2018 - 2028 MaiKe Labs
 *
 *  Library for STS3x digital temperature sensor
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

#ifndef __STS3X_H__
#define __STS3X_H__

#include <Arduino.h>

//#define	DEBUG				1

#ifdef DEBUG
#define	INFO(x)				Serial.println(x)
#else
#define	INFO(x)
#endif

#define	STS3X_ADDR_LOW				0x4A
#define	STS3X_ADDR_HIGH				0x4B

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

float sts3x_get_temp(void);
void sts3x_show_alert();

uint8_t sts3x_reset(void);
uint8_t sts3x_init(uint8_t scl, uint8_t sda);
#endif
