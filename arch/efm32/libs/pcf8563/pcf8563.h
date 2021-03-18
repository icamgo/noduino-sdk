/*
 *  Copyright (c) 2016 - 2026 MaiKe Labs
 *
 *  Library for PCF8563 RTC Chip
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
#ifndef __PCF8563_H__
#define __PCF8563_H__

#include <Arduino.h>
#include "softi2c.h"

typedef struct DateTime {
	uint8_t yOff;
	uint8_t m;
	uint8_t d;
	uint8_t hh;
	uint8_t mm;
	uint8_t ss;
} DateTime_t;

// 32-bit times as seconds since 1/1/1970
uint32_t dt_unixtime(void);

void pcf8563_init(uint8_t scl, uint8_t sda)
void pcf8563_set(const DateTime_t *dt);
void pcf8563_set_from_seconds(uint32_t t);
void pcf8563_set_from_str(const char* date, const char* time);
void pcf8563_set_from_int(uint16_t y, uint8_t m, uint8_t d,
							uint8_t hh, uint8_t mm, uint8_t ss);

uint32_t pcf8563_now();
void pcf8563_print();

#endif
