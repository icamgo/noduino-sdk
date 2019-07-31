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

#ifndef __MH_Z16_H__
#define __MH_Z16_H__

#include "Arduino.h"

void mhz16_begin(Stream * stream);

int32_t mhz16_get_co2();
int8_t mhz16_get_temp();

void mhz16_set_zero();
void mhz16_set_span_point();


void mhz16_send_cmd(uint8_t * cmd);
uint8_t mhz16_checksum(uint8_t * pkt);
#endif
