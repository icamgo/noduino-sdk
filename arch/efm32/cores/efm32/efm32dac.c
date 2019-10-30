/*
  EMF32ARDUINO efm32dac.c
  
  Copyright (c) 2018 huaweiwx@sina.com 2018.7.1

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_dac.h"

#include "efm32_build_defines.h"
#include "variant.h"
#include "wiring_constants.h"
#include "efm32gpio.h"

static uint32_t dacRef = dacRef1V25; /* dacRef1V25 / dacRef2V5 / dacRefVDD;*/

void setDacRef(uint32_t ref){
	dacRef = ref;
}

void initDac(uint32_t channel)
{
  DAC_Init_TypeDef        init        = DAC_INIT_DEFAULT;
  DAC_InitChannel_TypeDef initChannel = DAC_INITCHANNEL_DEFAULT;

  CMU_ClockEnable(cmuClock_DAC0, true);
  init.prescale = DAC_PrescaleCalc(1000000, 0);
  init.reference = dacRef;
  DAC_Init(DAC0, &init);
  DAC_InitChannel(DAC0, &initChannel, channel);
  DAC_Enable(DAC0, channel, true);
}

uint32_t dacValueCalcul(float vOut, float vRef)
{
  return (uint32_t)((vOut * 4095) / vRef);
}

uint32_t dacValueCalculRef(float vOut)
{
  switch (dacRef){
	 case  dacRef2V5:
	      return dacValueCalcul(vOut, 2.5);
  	 case  dacRefVDD:
	      return dacValueCalcul(vOut, 3.3);
	default:
	     break;
  }	
  return dacValueCalcul(vOut, 1.25);
}

/***************************************************************
 *    Continuously output dacValue to DAC channel
 ***************************************************************/
void  dacWrite(uint32_t channel,uint32_t dacValue){
  initDac(channel);
  DAC_ChannelOutputSet(DAC0, channel, dacValue);
}

