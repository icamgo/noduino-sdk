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

#define logo_width		126
#define logo_height		23

static uint8_t logo_xbm[] = {
   0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x06, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x1f, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x0e, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xf8, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0xc0, 0x01, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xfe, 0xfe, 0x01, 0x00, 0xc0, 0x01, 0x00, 0x00,
   0x1e, 0xc0, 0x01, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x7e, 0xf8, 0x03, 0x00,
   0xc0, 0x01, 0xc0, 0x00, 0x1e, 0xc0, 0x01, 0x0c, 0x00, 0x18, 0xc0, 0x03,
   0x1e, 0xe0, 0xf3, 0x00, 0xc7, 0x1f, 0xf0, 0x03, 0x1e, 0xc0, 0x01, 0x3f,
   0x00, 0xfe, 0xc0, 0x1f, 0x0e, 0xc0, 0xf3, 0x00, 0xc7, 0x1f, 0xfc, 0x0f,
   0x1e, 0xc0, 0xe1, 0x7f, 0x80, 0xff, 0xc1, 0x1f, 0x0e, 0xc0, 0xf3, 0x00,
   0xc7, 0x1f, 0xff, 0x3f, 0x1e, 0xc0, 0xf1, 0xff, 0xe3, 0xff, 0xc7, 0x1f,
   0x0e, 0xc0, 0xf3, 0x00, 0xc7, 0x1f, 0x3f, 0x7f, 0x1e, 0xe0, 0xf1, 0xf3,
   0xf3, 0xe7, 0xc7, 0x1f, 0x0e, 0xc0, 0xf3, 0x00, 0xc7, 0x01, 0x1f, 0x7e,
   0xfe, 0xff, 0xf1, 0x80, 0xf3, 0xc3, 0xc7, 0x03, 0xde, 0xdb, 0xf3, 0x00,
   0xc7, 0x01, 0x07, 0x78, 0xfe, 0xff, 0x71, 0x9c, 0xf3, 0x00, 0xc7, 0x03,
   0xfe, 0xff, 0xf3, 0x00, 0xc7, 0x01, 0x03, 0x78, 0xfe, 0xff, 0xf1, 0xff,
   0xf3, 0x00, 0xc7, 0x03, 0xfe, 0xff, 0xf3, 0x00, 0xc7, 0x01, 0x03, 0x78,
   0x1e, 0xe0, 0xf1, 0xff, 0xf3, 0x00, 0xc7, 0x03, 0xfe, 0xff, 0xf3, 0x00,
   0xc7, 0x01, 0x03, 0x78, 0x1e, 0xc0, 0xf1, 0xff, 0xf3, 0x00, 0xc7, 0x03,
   0x0e, 0xc0, 0xf3, 0x00, 0xc7, 0x01, 0x07, 0x78, 0x1e, 0xc0, 0x71, 0x00,
   0xf0, 0x00, 0xc7, 0x03, 0x0e, 0xc0, 0xf3, 0x81, 0xc7, 0x01, 0x07, 0x78,
   0x1e, 0xc0, 0x71, 0x00, 0xf0, 0x00, 0xc7, 0x03, 0x0e, 0xc0, 0xf3, 0xe7,
   0xc7, 0x07, 0x0f, 0x7e, 0x1e, 0xc0, 0xf1, 0xe0, 0xf0, 0xc3, 0xc7, 0x07,
   0x0e, 0xc0, 0xe3, 0xff, 0xc7, 0x1f, 0xff, 0x3f, 0x1e, 0xc0, 0xf1, 0xff,
   0xe3, 0xff, 0xc7, 0x1f, 0x0e, 0xc0, 0x83, 0xff, 0x83, 0x1f, 0xfc, 0x0f,
   0x1e, 0xc0, 0xe1, 0xff, 0x80, 0xff, 0x07, 0x1f, 0x0c, 0xc0, 0x01, 0xfe,
   0x00, 0x1e, 0xf0, 0x03, 0x1e, 0xc0, 0x80, 0x3f, 0x00, 0xfe, 0x07, 0x1c,
   0x08, 0x40, 0x00, 0x3c, 0x00, 0x18, 0xc0, 0x00, 0x18, 0x60, 0x00, 0x1e,
   0x00, 0x18, 0x04, 0x10, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#endif