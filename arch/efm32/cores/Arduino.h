/******************************************************************************
 * The MIT License
 *
 * EFM32 ARDUINO Copyright (c) 2018 huaweiwx<huaweiwx@sina.com>
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

#ifndef __ARDUINO_H__
#define __ARDUINO_H__

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "wiring_constants.h"
#include "efm32.h"

/*C including option*/
#include "bit_constants.h"

void setup();
void loop();

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus
void yield(void);    /*hooks.c*/
uint32_t shiftIn(uint32_t ulDataPin, uint32_t ulClockPin, uint32_t ulBitOrder );
void shiftOut( uint32_t ulDataPin, uint32_t ulClockPin, uint32_t ulBitOrder, uint8_t uVal );

#ifdef __cplusplus
}

long random(long);
long random(long, long);
void randomSeed(unsigned long);
long map(long, long, long, long, long);

#include <efm32/HardwareSerial.h>

#if USE_ARDUINOSTREAMING >0
#  include <Streaming.h>
#endif

#endif // __cplusplus

#if USE_BITBAND > 0
#include "util/bitband.h"
#endif

#endif
