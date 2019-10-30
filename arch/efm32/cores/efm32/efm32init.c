/*
  Copyright (c) 2018 huaweiwx@sina.com 2018.7.1

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
*/

#include <stddef.h>
#include "efm32.h"

void yield(void);
void ClockUpdate(void);
void RTC_Trigger(uint32_t msec, void(*funcPointer)(void));
uint16_t CRC_calc(uint8_t *start, uint8_t *end);

volatile uint32_t msTicks;

void systicCallback(void)  __attribute__ ((weak));
void systicCallback(void){}

#if (FREERTOS == 0)
void SysTick_Handler(void)
{
	++msTicks;
	systicCallback();   
}
#endif

void delay(uint32_t dlyTicks)
{
  uint32_t curTicks;
  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) yield();
}

void Fatal_Handler(void)
{
	while(1);
//    errorFlash(ERR_HardFault);
}

/**************************************************************************//**
 * @brief	Callback function for RTC
 *****************************************************************************/
volatile uint32_t Seconds;
void ClockUpdate(void)
{
	++Seconds;
}

extern  void SystemClock_Config(void);
void init(void)
{
    CHIP_Init();

    SystemClock_Config(); 	

	if (SysTick_Config(SystemCoreClockGet() / 1000)){ //
	     Fatal_Handler();             // never return;
	}
	CMU_ClockEnable(cmuClock_GPIO, true);
}

