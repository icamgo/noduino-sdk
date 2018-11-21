/*
 *  Copyright (c) 2016 - 2025 MaiKe Labs
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
#ifndef __PARAM_H__
#define __PARAM_H__

/*
 * 1MB spi flash: 0x0FA000
 * 4MB spi flash: 0x3FA000
 */
#define PARAM_START_SEC		0xFA

struct dev_param {
	float temp;
	float humi;
	float light;
	float press;
	uint32_t ch4;
	float v0;					/* init voltage of bridge, in mV */
	uint8_t relay;
	uint8_t realtime;			/* 1: mqtt enable, 0: mqtt disable */
	uint8_t airkiss_nff_on;
	uint8_t pack[2];
} __attribute__((aligned(4), packed));

uint8_t param_get_realtime();
void param_set_realtime(uint8_t d);
void param_save();
void param_erase_all();
void param_init();

#endif
