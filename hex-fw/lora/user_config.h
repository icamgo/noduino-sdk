/*
 *  Copyright (c) 2017 - 2025 MaiKe Labs
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
#ifndef __USER_CONFIG__
#define __USER_CONFIG__

#include <avr/io.h>
#include <avr/sleep.h>
#include <stdio.h>

#define F_CPU 8000000UL
#include <util/delay.h>

#include "wiring.h"
#include "spi.h"
#include "sx1278.h"

/* 
 * ATMEL ATTINY13
 *
 *                  +-\/-+
 * AinX (D X) PB5  1|    |8  Vcc
 * AinX (D X) PB3  2|    |7  PB2 (D X)  AinX
 * AinX (D X) PB4  3|    |6  PB1 (D X) PWM
 *            GND  4|    |5  PB0 (D X) PWM
 *                  +----+
*/
#define CS			0	
#define MOSI		1
#define MISO		2
#define SCK			5
#define PWR			3

#endif
