/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  fork from teensy3  huaweiwx@sina.com 2018.7.22  
*/

extern "C" {
  #include "stdlib.h"
  #include "stdint.h"
}


#if 1  /*select 1 for use new or 0 use older huaweiwx@sina.com 2017.7.22*/

/*new from teensy3*/

static long seed;
void randomSeed(uint32_t newseed)
{
	if (newseed > 0) seed =  (long) newseed;
}

void srandom(long newseed)
{
	if (newseed != 0) seed = newseed;
}

long random(void)
{
	long hi, lo, x;

    // the algorithm used in avr-libc 1.6.4
	x = seed;
	if (x == 0) x = 123459876;
	hi = x / 127773;
	lo = x % 127773;
	x = 16807 * lo - 2836 * hi;
	if (x < 0) x += 0x7FFFFFFF;
	seed = x;
	return x;
}

long random(long howbig)
{
	if (howbig == 0) return 0;
	return random() % howbig;
}

#else
/* old */
extern void randomSeed( uint32_t dwSeed )
{
  if ( dwSeed != 0 )
  {
    srand( dwSeed ) ;
  }
}

extern long random( long howbig )
{
  if ( howbig == 0 )
  {
    return 0 ;
  }

  return rand() % howbig;
}

#endif


extern long random( long howsmall, long howbig )
{
  if (howsmall >= howbig)
  {
    return howsmall;
  }

  long diff = howbig - howsmall;

  return random(diff) + howsmall;
}

extern long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

extern uint16_t makeWord( uint16_t w )
{
  return w ;
}

extern uint16_t makeWord( uint8_t h, uint8_t l )
{
  return (h << 8) | l ;
}
