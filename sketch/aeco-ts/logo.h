/* 
 *  Copyright (c) 2019 - 2029 MaiKe Labs
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#ifndef __LOGO_H__
#define __LOGO_H__

#define c_width 17
#define c_height 13

static uint8_t c_icon[] = {
   0x00, 0x00, 0x00, 0x3c, 0x78, 0x00, 0x66, 0xfe, 0x00, 0x66, 0x07, 0x00,
   0x66, 0x03, 0x00, 0x66, 0x03, 0x00, 0x3c, 0x01, 0x00, 0x00, 0x03, 0x00,
   0x00, 0x03, 0x00, 0x00, 0x07, 0x00, 0x00, 0xfe, 0x00, 0x00, 0x78, 0x00,
   0x00, 0x00, 0x00 };

#define logo_width		102
#define logo_height		59

static uint8_t logo_xbm[] = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x7f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x07, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x80, 0x1f,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x07, 0x00,
   0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x01,
   0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70,
   0x00, 0x1e, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xe0, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xfc, 0xe1, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x1f, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xf1, 0x03, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x03, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
   0x00, 0x00, 0x20, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x0f, 0x00,
   0x00, 0x00, 0x00, 0xe0, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x3f,
   0x00, 0x00, 0x00, 0x00, 0xe0, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0xfc,
   0xfc, 0x00, 0x80, 0x03, 0x00, 0xe0, 0x80, 0x03, 0x00, 0x00, 0x80, 0x03,
   0x3c, 0xf0, 0x1c, 0x9c, 0x1f, 0x78, 0xe0, 0x80, 0x03, 0x07, 0xe0, 0x81,
   0x0f, 0x1c, 0xe0, 0x1c, 0x9c, 0x1f, 0xfe, 0xe1, 0x80, 0xc3, 0x1f, 0xf8,
   0x87, 0x0f, 0x1c, 0xe0, 0x1c, 0x9c, 0x9f, 0xff, 0xe7, 0x80, 0xf3, 0x7f,
   0xfe, 0x9f, 0x0f, 0x1c, 0xe0, 0x1c, 0x9c, 0x83, 0x87, 0xe7, 0xff, 0xf3,
   0x78, 0x3e, 0x9f, 0x03, 0xfc, 0xff, 0x1c, 0x9c, 0x83, 0x03, 0xe7, 0xff,
   0x73, 0x70, 0x0e, 0x9c, 0x03, 0xfc, 0xff, 0x1c, 0x9c, 0x83, 0x03, 0xe7,
   0xff, 0xf3, 0x7f, 0x0e, 0x9c, 0x03, 0xfc, 0xff, 0x1c, 0x9c, 0x83, 0x03,
   0xe7, 0x80, 0xf3, 0x7f, 0x0e, 0x9c, 0x03, 0x1c, 0xe0, 0x1c, 0x9c, 0x83,
   0x03, 0xe7, 0x80, 0x73, 0x00, 0x0e, 0x9c, 0x03, 0x1c, 0xe0, 0x1c, 0x9c,
   0x83, 0x03, 0xe7, 0x80, 0x73, 0x00, 0x0e, 0x9c, 0x03, 0x1c, 0xe0, 0x3c,
   0x9e, 0x8f, 0xcf, 0xe7, 0x80, 0xf3, 0x79, 0x3e, 0x9f, 0x07, 0x1c, 0xe0,
   0xfc, 0x1f, 0x1f, 0xff, 0xe3, 0x80, 0xf3, 0x3f, 0xfc, 0x9f, 0x0f, 0x18,
   0x60, 0xf0, 0x07, 0x1c, 0xfc, 0xc0, 0x80, 0xc1, 0x1f, 0xf0, 0x1f, 0x0e,
   0x10, 0x20, 0xc0, 0x01, 0x10, 0x78, 0x80, 0x80, 0x00, 0x07, 0xc0, 0x18,
   0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
   0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xf0, 0xf1, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xc0, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x03, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xe1, 0x0f, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x1e, 0x80, 0x03, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x00, 0xe0, 0x01,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x07, 0x00, 0xf8,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x80,
   0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8,
   0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x80, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#define no_sensor_width 91
#define no_sensor_height 52
static unsigned char no_sensor_icon[] = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xf0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xfc, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xfc, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x1c, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfc, 0x1f, 0x0e, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfe, 0x1f, 0x1e, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfe, 0x1f, 0x1e, 0x00, 0xe0, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfe, 0x1f, 0xfe, 0x01, 0xc0, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xce, 0x01, 0xfe, 0x03, 0xc0, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xce, 0x01, 0xfc, 0x07, 0xc0, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xce, 0x01, 0xc0, 0x0f, 0x80, 0xc1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xcf, 0x01, 0x00, 0x0f, 0x80, 0xc1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xcf, 0x01, 0x00, 0x0f, 0x80, 0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xcf, 0x01, 0x00, 0x0f, 0x00, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0xef, 0x1f, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0xef, 0x1f, 0x18, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0xef, 0x3f, 0x00, 0xfc, 0x01, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0x80, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0x03, 0xf8, 0xff, 0x07, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0xff, 0x3f, 0x03, 0x00, 0x80, 0xff, 0x1f, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0xff, 0x3f, 0x03, 0x00, 0x00, 0x00, 0xff, 0x1f, 0x00,
   0xc7, 0x01, 0x00, 0xff, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0x01,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0xf8, 0xff, 0x07, 0x00, 0x00, 0x00, 0x03,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0x00, 0x80, 0xff, 0x1f, 0x00, 0x00, 0x03,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0x00, 0x00, 0x00, 0xff, 0x7f, 0x00, 0x03,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x03,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x01,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0x7f, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0xef, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0xef, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc7, 0x01, 0x00, 0xef, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xcf, 0x01, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xcf, 0x01, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xcf, 0x01, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xce, 0x01, 0xf0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xce, 0x01, 0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfe, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfe, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfe, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfc, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfc, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#define x_high_width 69
#define x_high_height 59
static unsigned char x_high_icon[] = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0x01,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x1f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xe0, 0x1f, 0x0c, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8,
   0x00, 0x0c, 0xe0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x0c, 0x00,
   0x07, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x0c, 0x00, 0x1e, 0x00, 0x00,
   0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0xe0, 0x00,
   0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00,
   0xc0, 0x01, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00,
   0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x6e, 0x00,
   0x00, 0x00, 0x00, 0x80, 0x0d, 0x00, 0x00, 0xe7, 0x00, 0x00, 0x00, 0x00,
   0xc0, 0x0d, 0x00, 0x80, 0xc3, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x18, 0x00,
   0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x60, 0x30, 0x00, 0xc0, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00,
   0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x30, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x80, 0x01, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x08, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x02, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06,
   0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x04, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x0c, 0x06, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x0c,
   0x06, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x0c, 0x06, 0x00, 0x00,
   0x00, 0x3f, 0x00, 0x00, 0x00, 0x0c, 0xf6, 0x00, 0x00, 0x00, 0xfe, 0x00,
   0x00, 0xe0, 0x0d, 0xf4, 0x00, 0x00, 0x00, 0xcc, 0x03, 0x00, 0xe0, 0x05,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x3c, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0,
   0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x03, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x3c, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
   0xf0, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0xc0, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
   0x00, 0x41, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x80, 0xc1, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x88, 0x00, 0x00, 0x80, 0x00,
   0x00, 0x00, 0x00, 0xc0, 0x88, 0x01, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x00,
   0x40, 0x08, 0x01, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x20, 0x08, 0x02,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x08, 0x06, 0x00, 0x60, 0x00,
   0x00, 0x00, 0x00, 0x10, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x08, 0x08, 0x08, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x08,
   0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x0c, 0x00, 0x08, 0x00,
   0x00, 0x00, 0x00, 0xf0, 0xff, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00 };

#define x_low_width 69
#define x_low_height 59
static unsigned char x_low_icon[] = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff, 0x01,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x1f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xe0, 0x07, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8,
   0x00, 0x06, 0xe0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x06, 0x00,
   0x07, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x06, 0x00, 0x1e, 0x00, 0x00,
   0x00, 0xc0, 0x03, 0x00, 0x06, 0x00, 0x38, 0x00, 0x00, 0x00, 0xe0, 0x00,
   0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00,
   0xc0, 0x01, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00,
   0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x36, 0x00,
   0x00, 0x00, 0x00, 0xc0, 0x0e, 0x00, 0x00, 0x77, 0x00, 0x00, 0x00, 0x00,
   0xe0, 0x0c, 0x00, 0x80, 0xe3, 0x00, 0x00, 0x00, 0x00, 0x60, 0x18, 0x00,
   0x80, 0xc1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0xc0, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00,
   0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x30, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x80, 0x01, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x08, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x02, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06,
   0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x04, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x0e, 0x06, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x0c,
   0x06, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x0c, 0x06, 0x00, 0x00,
   0x80, 0x0f, 0x00, 0x00, 0x00, 0x0c, 0xf6, 0x00, 0x00, 0xe0, 0x0f, 0x00,
   0x00, 0xe0, 0x0d, 0xf6, 0x00, 0x00, 0xf8, 0x06, 0x00, 0x00, 0xe0, 0x0d,
   0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0xc0,
   0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0xe0, 0x01, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0xc0, 0x03,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xf0, 0x00, 0x00, 0x08, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00,
   0x08, 0x00, 0x00, 0x00, 0x23, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00,
   0x00, 0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x41, 0x00,
   0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x80, 0xc8, 0x00, 0x00, 0x00, 0x00,
   0x70, 0x00, 0x00, 0xc0, 0x88, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00,
   0x40, 0x88, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x08, 0x01,
   0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x30, 0x08, 0x02, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x10, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
   0x08, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x08, 0x00, 0x0c,
   0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x18, 0x00, 0x08, 0x00, 0x00, 0x00,
   0x00, 0x08, 0x00, 0xf0, 0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00 };

#define battery_width 18
#define battery_height 10
static unsigned char battery_icon[] = {
   0xff, 0xff, 0x00, 0x01, 0x80, 0x00, 0x49, 0x92, 0x01, 0x49, 0x92, 0x02,
   0x49, 0x92, 0x02, 0x49, 0x92, 0x02, 0x49, 0x92, 0x02, 0x49, 0x92, 0x01,
   0x01, 0x80, 0x00, 0xff, 0xff, 0x00 };

#define low_battery_width 18
#define low_battery_height 10
static unsigned char low_battery_icon[] = {
   0xff, 0xff, 0x00, 0x01, 0x80, 0x00, 0x09, 0x80, 0x01, 0x09, 0x80, 0x02,
   0x09, 0x80, 0x02, 0x09, 0x80, 0x02, 0x09, 0x80, 0x02, 0x09, 0x80, 0x01,
   0x01, 0x80, 0x00, 0xff, 0xff, 0x00 };

// battery --> 11px
// unit    --> 14px
// press   --> 47px, 31px
// devid   --> 11px or 12px

#define ptest_width 129
#define ptest_height 128
static unsigned char ptest[] = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0,
   0xaa, 0x02, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00,
   0x72, 0x00, 0x00, 0x00, 0x58, 0x92, 0x0c, 0x00, 0x00, 0x98, 0x01, 0x00,
   0x00, 0x00, 0x00, 0x84, 0x80, 0x41, 0x00, 0x00, 0x00, 0x08, 0x00, 0x08,
   0x00, 0x00, 0x18, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x40, 0x00,
   0x00, 0x00, 0x48, 0x00, 0x38, 0x00, 0x00, 0x18, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x60, 0xc0, 0x43, 0x44, 0x00, 0x00, 0x48, 0x00, 0x28, 0x00, 0x00,
   0x98, 0xe1, 0xa5, 0x01, 0x00, 0x00, 0x60, 0x40, 0x44, 0x48, 0x00, 0x00,
   0x48, 0x00, 0x28, 0x00, 0x00, 0xf8, 0x31, 0x66, 0x01, 0x00, 0x00, 0x80,
   0x20, 0x64, 0x28, 0x00, 0x00, 0x48, 0x00, 0x48, 0x00, 0x00, 0x18, 0x13,
   0x26, 0x00, 0x00, 0x00, 0x80, 0x20, 0x44, 0x30, 0x00, 0x00, 0x48, 0x00,
   0x28, 0x00, 0x00, 0x18, 0x1a, 0x2c, 0x00, 0x00, 0x00, 0x4c, 0x60, 0x42,
   0x10, 0x00, 0x00, 0x48, 0x00, 0x18, 0x00, 0x00, 0x18, 0x12, 0x24, 0x00,
   0x00, 0x00, 0x78, 0xc4, 0x43, 0x10, 0x00, 0x00, 0x08, 0x00, 0x08, 0x00,
   0x00, 0x18, 0x13, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xf8, 0xff, 0x0f, 0x00, 0x00, 0xd8, 0xb1, 0x26, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x55, 0x01, 0x00, 0x00, 0x78,
   0xe1, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x7f, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
   0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0x03,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xc0, 0xff, 0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x1f, 0xf0, 0x0f, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0,
   0x0f, 0xe0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xe0, 0x07, 0xc0, 0x1f, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x07, 0x80,
   0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xf0, 0x03, 0x80, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0x80, 0x1f, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x80, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x1f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x80, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x0f,
   0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x94, 0x02, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xe0, 0x0f, 0x00, 0x00, 0x00, 0xfe, 0x07, 0x00, 0xfe,
   0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x07, 0x00, 0x00,
   0x00, 0xff, 0x0f, 0x00, 0xfe, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x80, 0xff, 0x07, 0x00, 0x00, 0x80, 0xaf, 0x1f, 0x00, 0xff, 0x0f, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x03, 0x00, 0x00, 0xc0, 0x07,
   0x1e, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff,
   0x01, 0x00, 0x00, 0xc0, 0x03, 0x1c, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x80, 0xff, 0x01, 0x00, 0x00, 0xc0, 0x03, 0x3c, 0x80,
   0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x07, 0x00,
   0x00, 0x40, 0x00, 0x3c, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xfc, 0x07, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x80, 0x03, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x0f, 0x00, 0x00, 0x00,
   0x00, 0x3c, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc0, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x1e, 0xc0, 0xff, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x1e,
   0xc0, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x1f,
   0x00, 0x00, 0x00, 0xe0, 0x0f, 0xc0, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0xe0, 0x07, 0xe0, 0xc0,
   0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00,
   0x00, 0xe0, 0x0f, 0x20, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0xa0, 0x0f, 0x00, 0x00, 0x0f, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00,
   0x1e, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00,
   0x3f, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xf8, 0x01, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00,
   0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x01, 0x80, 0x1f, 0x00,
   0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xf8, 0x03, 0x80, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x1e,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0xc0, 0x1f, 0x00, 0x00, 0x80,
   0x00, 0x38, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x0f,
   0xe0, 0x0f, 0x00, 0x0e, 0xe0, 0x01, 0x3c, 0x10, 0x00, 0x0f, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xe0, 0x3f, 0xf8, 0x0f, 0x00, 0x3f, 0xe0, 0x01, 0x3c,
   0x38, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff, 0x07,
   0x00, 0x3f, 0xc0, 0x03, 0x1e, 0x7c, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xc0, 0xff, 0xff, 0x03, 0x80, 0x3f, 0xc0, 0x8f, 0x1f, 0xf8, 0xe1,
   0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0x01, 0x80, 0x7f,
   0x80, 0xff, 0x0f, 0xf0, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfe, 0xff, 0x00, 0x00, 0x3f, 0x00, 0xff, 0x07, 0xe0, 0xff, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x1f, 0x00, 0x00, 0x1f, 0x00, 0xfc,
   0x01, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x02,
   0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x20, 0x30, 0xc0, 0x80, 0x01, 0x00, 0x04, 0x04, 0x0c, 0x04,
   0x10, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0xd8, 0x20, 0x41, 0x02,
   0x06, 0x1b, 0x0c, 0x12, 0x0c, 0x08, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x60, 0x88, 0x10, 0x23, 0x04, 0x02, 0x11, 0x0c, 0x21, 0x08, 0x08, 0x44,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x80, 0x11, 0x22, 0x04, 0x81, 0x20,
   0x08, 0x21, 0x08, 0x04, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x80,
   0x08, 0x32, 0x04, 0x83, 0x20, 0x0c, 0x21, 0x08, 0x0e, 0x64, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x20, 0x80, 0x18, 0x22, 0x88, 0x8d, 0x20, 0x88, 0x41,
   0x08, 0x36, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x40, 0x08, 0x32,
   0x44, 0x88, 0x20, 0x0c, 0x21, 0x08, 0x21, 0x44, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x20, 0x20, 0x10, 0x22, 0x44, 0x90, 0x21, 0x08, 0x21, 0x08, 0x21,
   0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x10, 0x10, 0x22, 0x44, 0x08,
   0x31, 0x0c, 0x21, 0x08, 0x21, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
   0x58, 0x30, 0x41, 0x86, 0x08, 0x13, 0x08, 0x32, 0x18, 0x33, 0x66, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x60, 0xf8, 0xc1, 0xc0, 0x81, 0x07, 0x0e, 0x0c,
   0x0e, 0x08, 0x0c, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00 };
#endif