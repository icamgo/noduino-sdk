/*
  main.cpp - Main loop for Arduino sketches
  Copyright (c) 2005-2013 Arduino Team.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


/*GCC tools chain check  add by huaweiwx@sina.com 2018.9.7*/
#if __GNUC__ > 5 || (__GNUC__ == 5 && (__GNUC_MINOR__ > 4 || \
   (__GNUC_MINOR__ == 4 && __GNUC_PATCHLEVEL__ > 0)))

#include "arduino.h"

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (* /*func*/ )()) { return 0; }

void initVariant() __weak;
void initVariant() { }

extern "C" void init(void);
extern void setup(void);
extern void  loop(void);

int main()
{
	init();

	initVariant();

	setup();
    while(1)
    {
#if USE_CORECALLBACK > 0
      coreCallback();
#endif
	  loop();
#if USE_SERIALEVENTRUN > 0	
      if (serialEventRun) serialEventRun();
#endif
    }
//	return 1; //
}


#else
#error "Please update to GCC ver 5-2016q2 https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads"	
#endif
