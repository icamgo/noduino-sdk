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

#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "efm32_build_defines.h"
#include "variant.h"
#include "wiring_constants.h"
#include "efm32gpio.h"

// Note: change this to set the desired output frequency in Hz
#define PWM_FREQ 1000


// Note: change this to set the desired duty cycle (used to update CCVB value)
static int dutyCyclePercent[4]={50,50,50,50};
static int channel[4] = {0,0,0,0};

/**************************************************************************//**
 * @brief
 *    Interrupt handler for TIMER1 that changes the duty cycle
 *
 * @note
 *    This handler doesn't actually dynamically change the duty cycle. Instead,
 *    it acts as a template for doing so. Simply change the dutyCyclePercent
 *    global variable here to dynamically change the duty cycle.
 *****************************************************************************/
#if USE_TIMER0_PWM >0
void TIMER0_IRQHandler(void)
{
  // Acknowledge the interrupt
  uint32_t flags = TIMER_IntGet(TIMER0);
  TIMER_IntClear(TIMER0, flags);

  // Update CCVB to alter duty cycle starting next period
  TIMER_CompareBufSet(TIMER0, channel[0], (TIMER_TopGet(TIMER0) * dutyCyclePercent[0]) / 100);
}
#endif
#if USE_TIMER1_PWM >0
void TIMER1_IRQHandler(void)
{
  // Acknowledge the interrupt
  uint32_t flags = TIMER_IntGet(TIMER1);
  TIMER_IntClear(TIMER1, flags);

  // Update CCVB to alter duty cycle starting next period
  TIMER_CompareBufSet(TIMER1, channel[1], (TIMER_TopGet(TIMER1) * dutyCyclePercent[1]) / 100);
}
#endif
#if defined(TIMER2)&& (USE_TIMER2_PWM >0)
void TIMER2_IRQHandler(void)
{
  // Acknowledge the interrupt
  uint32_t flags = TIMER_IntGet(TIMER2);
  TIMER_IntClear(TIMER2, flags);

  // Update CCVB to alter duty cycle starting next period
  TIMER_CompareBufSet(TIMER2, channel[2], (TIMER_TopGet(TIMER2) * dutyCyclePercent[2]) / 100);
}
#endif
#if defined(TIMER3)&&(USE_TIMER3_PWM >0)
void TIMER3_IRQHandler(void)
{
  // Acknowledge the interrupt
  uint32_t flags = TIMER_IntGet(TIMER3);
  TIMER_IntClear(TIMER3, flags);

  // Update CCVB to alter duty cycle starting next period
  TIMER_CompareBufSet(TIMER3, channel[3], (TIMER_TopGet(TIMER3) * dutyCyclePercent[3]) / 100);
}
#endif
/**************************************************************************//**
 * @brief
 *    TIMER initialization
 *****************************************************************************/
static void initTimer(TIMER_TypeDef* TIMERx, unsigned int ch, unsigned int routeLoc, int dutyCyclePercentx)
{
  // Enable clock for TIMER1 module
  CMU_Clock_TypeDef cmuClock_TIMERx;
#if (USE_TIMER0_PWM >0)
  if(TIMERx == TIMER0){
      cmuClock_TIMERx = cmuClock_TIMER0;
	  dutyCyclePercent[0]=dutyCyclePercentx; 
      channel[0]=ch;	  
  }
#endif
#if USE_TIMER1_PWM >0  
  if(TIMERx == TIMER1){
      cmuClock_TIMERx = cmuClock_TIMER1;
	  dutyCyclePercent[1]=dutyCyclePercentx; 
      channel[1]=ch;	  
  }
#endif
#if defined(TIMER2)&& (USE_TIMER2_PWM >0)
  if(TIMERx == TIMER2){
      cmuClock_TIMERx = cmuClock_TIMER2;
	  dutyCyclePercent[2]=dutyCyclePercentx; 
      channel[2]=ch;	  
 }
#endif
#if defined(TIMER3)&&(USE_TIMER3_PWM >0)
  if(TIMERx == TIMER3){
      cmuClock_TIMERx = cmuClock_TIMER3;
	  dutyCyclePercent[3]=dutyCyclePercentx; 
      channel[3]=ch;	  
  }
#endif

  CMU_ClockEnable(cmuClock_TIMERx, true);
  // Configure TIMERx Compare/Capture for output compare
  // Use PWM mode, which sets output on overflow and clears on compare events
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
  timerCCInit.mode = timerCCModePWM;
  TIMER_InitCC(TIMERx, ch, &timerCCInit);

  // Route TIMERx CC0 to location 4 and enable CC0 route pin
  unsigned int routPin;
  switch (ch){
    case 1:
	  routPin = TIMER_ROUTE_CC1PEN;
	  break;
    case 2:
	  routPin = TIMER_ROUTE_CC2PEN;
	  break;
	default:
 	  routPin = TIMER_ROUTE_CC0PEN;  
      break;	  
  }
  
  TIMERx->ROUTE |= (routPin | routeLoc);

  // Set top value to overflow at the desired PWM_FREQ frequency
  TIMER_TopSet(TIMERx, CMU_ClockFreqGet(cmuClock_TIMERx) / PWM_FREQ);

  // Set compare value for initial duty cycle
  TIMER_CompareSet(TIMERx, ch, (TIMER_TopGet(TIMERx) * dutyCyclePercentx) / 100);

  // Initialize the timer
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  TIMER_Init(TIMERx, &timerInit);

  // Enable TIMERx compare event interrupts to update the duty cycle
  TIMER_IntEnable(TIMERx, TIMER_IEN_CC0);
#if USE_TIMER0_PWM >0
  if(TIMERx == TIMER0)  NVIC_EnableIRQ(TIMER0_IRQn);
#endif
#if USE_TIMER1_PWM >0
  if(TIMERx == TIMER1)  NVIC_EnableIRQ(TIMER1_IRQn);
#endif
#if defined(TIMER2)&& (USE_TIMER2_PWM >0)
  if(TIMERx == TIMER2)  NVIC_EnableIRQ(TIMER2_IRQn);
#endif
#if defined(TIMER3)&& (USE_TIMER1_PWM >0)
  if(TIMERx == TIMER3)  NVIC_EnableIRQ(TIMER3_IRQn);
#endif
}

void analogWrite(uint8_t ucPin, uint32_t val)
{
  TIMER_TypeDef *timer  = g_Pin2PortMapArray[ucPin].pTimer;
//  if((uint32_t*)timer == NO_PWM) return;
  uint32_t chLoc  = g_Pin2PortMapArray[ucPin].timerChannelLoc;
  pinMode(ucPin,OUTPUT);
  int duty = val * 100 / 0xfff;
  initTimer(timer, chLoc&0x0f, chLoc >> 4, duty);
}

