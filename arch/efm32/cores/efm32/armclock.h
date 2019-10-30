/*
  ARM CLOCK UTILS
  
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

#ifndef  __ARMCLOCK_H__
 #define __ARMCLOCK_H__

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

extern volatile uint32_t msTicks;

inline uint32_t millis() {
   return msTicks;	
}

inline uint32_t micros() {
  uint32_t m  = msTicks;
  uint32_t u  = SysTick->LOAD - SysTick->VAL;
  uint32_t m1 = msTicks;
  uint32_t u1 = SysTick->LOAD - SysTick->VAL;

  if (m1 > m) {
    return ( m1 * 1000 + (u1 * 1000) / SysTick->LOAD);
  } else {
    return ( m * 1000 + (u * 1000) / SysTick->LOAD);
  }
}

void delay(uint32_t dlyTicks);

inline void delayMicroseconds(uint32_t microseconds){
  uint32_t start = micros();

  while(start + microseconds > micros()) {
  }
}

#ifdef __cplusplus
}  //extern "C"{
#endif /* __cplusplus*/

/*10 cycle*/
#define _delay_loop_2(x)  do{for(uint32_t i=0;i<x;i++) {asm volatile("nop");}}while(0)

#endif /*__ARMCLOCK_H__*/