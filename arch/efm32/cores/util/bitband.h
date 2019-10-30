/* 
  huaweiwx@sina.com 2018.  All right reserved.

  for efm32 m3/m4 sram/peripheral bitband io opration
  add arduino style class BB_PIN 2017.12.20
  
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
 
//bitband io opration,实现51类似的GPIO控制功能

#ifndef __BITBAND_H__
#define __BITBAND_H__
#include "arduino.h"
#include "em_bitband.h"
#include "digitalPin.h"

#if defined(BITBAND_PER_BASE)
#define MEM_ADDR(addr)  *((volatile unsigned long  *)((uint32_t)addr)) 
#define BITBAND(addr, bitnum) (BITBAND_PER_BASE+(((uint32_t)addr)<<5)+(bitnum<<2)) 
#define BITBAND_ADDR(addr, bitnum)   MEM_ADDR(BITBAND((uint32_t)(addr), bitnum)) 

#ifdef __cplusplus
/*gpio bitband interface class*/
class BB_PIN{
 public:
     BB_PIN(__ConstPin CPin): CPin(CPin){};
       __ConstPin CPin;

	 const uint32_t inReg  =(const uint32_t)&GPIO->P[CPin.GPIOx_Port].DIN;
	 const uint32_t outReg =(const uint32_t)&GPIO->P[CPin.GPIOx_Port].DOUT;
	 
     const uint32_t bb_inadr = BITBAND(inReg,CPin.pinMask);
     const uint32_t bb_outadr = BITBAND(outReg,CPin.pinMask);
	 
 	 template<typename T = bool>
     inline  T read() {return MEM_ADDR(bb_inadr);}

 	 template<typename T>
     inline void write(T value){MEM_ADDR(this->bb_outadr) = value;}
  
 	 template<typename T>
     inline BB_PIN & operator = (T value){
       this->write(value);
       return *this;
     }
 	 template<typename T>
     inline BB_PIN & operator ^= (T value){
       if(value)this->toggle();
       return *this;
     }
  
     BB_PIN& operator = (BB_PIN& rhs) {
       this->write(rhs.read());
       return *this;
     }
  
    inline __attribute__((always_inline))
    void high(){MEM_ADDR(this->bb_outadr) = 0x1U;}

    inline __attribute__((always_inline))
    void low(){MEM_ADDR(this->bb_outadr) = 0x0U;}

    template<typename T> 
    inline operator T (){
      return (MEM_ADDR(this->bb_inadr));
    }

	inline void operator  !() __attribute__((always_inline)) {
      toggle();
    }

/*----- comptabled with DigitalPin ----------*/
    inline __attribute__((always_inline))
    void toggle(){MEM_ADDR(this->bb_outadr) ^= 1;}

    inline __attribute__((always_inline))
    void config(uint32_t mode, bool value) {  /*compatale with digitalPin*/
	  this->mode(mode);
      this->write(value);
    }

    inline __attribute__((always_inline))
    void mode(uint32_t mode){
           pinMode(CPin,mode);
	}
	  
    inline __attribute__((always_inline))
    void attach(voidFuncPtr callback, uint32_t mode){
	   attachInterrupt(CPin, callback, mode);
    }
  
  inline __attribute__((always_inline))
  void detach(void){
	  detachInterrupt(CPin);
  }
};
#endif //__cplusplus
#endif //BITBAND_PER_BASE

#ifdef __cplusplus
#  ifndef BITBAND_PER_BASE
#    define BB_PIN  LL_PIN
#   endif
#endif //__cplusplus

#endif //__BITBAND_H__
