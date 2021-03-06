/*
 * Copyright (c) 2015 - 2025 MaiKe Labs
 * 
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at your
 * option) any later version.
 * 
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef __TWI_H__
#define __TWI_H__ 

#include "Arduino.h"

//#define	DEBUG				1

#ifdef DEBUG
#define	INFO(x)				Serial.println(x)
#else
#define	INFO(x)
#endif

#if	0
#define I2C_DELAY			5		/* us delay */
#define I2C_MAXWAIT			800
#define i2c_delay(x)		delayMicroseconds(x)
#else
#define I2C_DELAY			(F_CPU/100000)		/* 14MHz, 1Tick = 1/14 us, 10us */
#define I2C_MAXWAIT			800
#define i2c_delay(x) do{for(int i=0;i<x;i++) {asm volatile("nop");}}while(0)
#endif

void wire_begin(uint8_t scl, uint8_t sda);
void wire_setClock(uint32_t clk);
void wire_beginTransmission(uint8_t addr);
size_t wire_write(uint8_t data);
uint8_t wire_endTransmission();
uint8_t wire_requestFrom(uint8_t addr, size_t len);
int wire_read();
int wire_available(void);
void wire_end();

bool i2c_init();
void i2c_deinit();
bool i2c_start();
bool i2c_stop();
void i2c_ack();

bool i2c_write_byte(uint8_t byte);
uint8_t i2c_read_byte(bool nack);

#endif
