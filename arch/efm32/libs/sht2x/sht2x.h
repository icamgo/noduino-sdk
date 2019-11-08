/*
 *  Copyright (c) 2016 - 2026 MaiKe Labs
 *
 *  Library for SHT21/SHT2x digital temperature sensor
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

#ifndef __SHT2X_H__
#define __SHT2X_H__

#include <Arduino.h>

#define	SHT2X_ADDR			0x40

#define	T_HOLD_CMD			0xE3
#define	RH_HOLD_CMD			0xE5
#define	WRITE_USER_REG		0xE6
#define	READ_USER_REG		0xE7
#define	T_NO_HOLD_CMD		0xF3
#define	RH_NO_HOLD_CMD		0xF5
#define SOFT_RESET			0xFE

float sht2x_get_humi(void);
float sht2x_get_temp(void);
void sht2x_set_res(uint8_t r);
uint8_t sht2x_reset(void);
uint8_t sht2x_init(void);
#endif
