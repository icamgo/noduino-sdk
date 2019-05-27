/*
 *  Copyright (c) 2015 - 2025 MaiKe Labs
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
#ifndef __TWI_H__
#define __TWI_H__ 

#include "noduino.h"

static const uint8_t SDA = 4;
static const uint8_t SCL = 5;

void wire_begin();
void wire_setClock(uint32_t clk);
void wire_beginTransmission(uint8_t addr);
size_t wire_write(uint8_t data);
uint8_t wire_endTransmission();
uint8_t wire_requestFrom(uint8_t addr, size_t len);
int wire_read();

void twi_init (unsigned char sda, unsigned char scl);
void twi_stop (void);
void twi_setClock (unsigned int freq);
uint8_t	twi_writeTo (unsigned char address, unsigned char *buf, unsigned int len, unsigned char sendStop);
uint8_t	twi_readFrom (unsigned char address, unsigned char *buf, unsigned int len, unsigned char sendStop);

#endif
