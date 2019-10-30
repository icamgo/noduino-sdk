/*
  EMF32 ARDUINO  avr_emulation.h
  
  Copyright (c) 2018 huaweiwx@sina.com 2018.9.1

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


#ifndef __AVREMULATION_H__
#define __AVREMULATION_H__

#ifdef __cplusplus

class PINemulation 
{
  public:
	PINemulation(const GPIO_Port_TypeDef port): port(port){}
	const GPIO_Port_TypeDef port;

    inline  operator uint32_t () const __attribute__((always_inline)){
		return GPIO->P[port].DIN;
    }
};

class PORTemulation 
{
  public:
	PORTemulation(const GPIO_Port_TypeDef port): port(port){}
	const GPIO_Port_TypeDef port;
	
    inline  operator uint32_t () const __attribute__((always_inline)){
		return GPIO->P[port].DOUT;
    }

    inline PORTemulation& operator = (uint16_t val) __attribute__((always_inline)){
        GPIO->P[port].DOUT = val;
        return *this;
	}
    inline PORTemulation& operator |= (uint16_t val) __attribute__((always_inline)) {
		GPIO_PortOutSet(port,val);
        return *this;
	}
    inline PORTemulation& operator &= (uint16_t val) __attribute__((always_inline)) {
		GPIO_PortOutClear(port,~val);
        return *this;
	}
    inline PORTemulation& operator ^= (uint16_t val) __attribute__((always_inline)) {
		GPIO_PortOutToggle(port,val);
        return *this;
	}
};

class DDRemulation
{
  public:
    DDRemulation(const GPIO_Port_TypeDef port, uint16_t mask): port(port),mask(mask){}
	const GPIO_Port_TypeDef port;
	uint16_t mask;

	inline void setMask(uint16_t val){
		mask = val;
	}

	inline uint16_t getMask(void){
		return mask;
	}

	inline void lock(uint16_t pinMask){
		this->mask &= ~pinMask;
	}

	inline void unLock(uint16_t pinMask){
		this->mask |= pinMask;
	}

    void mode(unsigned int pinMask, GPIO_Mode_TypeDef mode, unsigned int out = 0) {
		GPIO_PinModeSet(port,pinMask,mode,out);
    }

    inline operator int () __attribute__((always_inline)) {
		return this->save_ddr;
    }

    DDRemulation& operator = (uint16_t val){
      this->save_ddr = val & mask;
	  for(uint8_t i = 0;i<16;i++){
		  if(bitRead(mask,i)){
			GPIO_PinModeSet(port,i, bitRead(val,i)?gpioModePushPull:gpioModeInput, 0);
 	      }
	  }
      return *this;
    }

    inline DDRemulation& operator |= (uint16_t val) __attribute__((always_inline)) {
	  for(uint8_t i = 0;i<16;i++){
		  if((bitRead(mask,i))&(~save_ddr)){
			 if(bitRead(val,i)){
			    GPIO_PinModeSet(port,i,gpioModePushPull,0);
				bitSet(save_ddr,i);
 	         }			 
		  }
	  }
      return *this;
    }
	
    inline DDRemulation& operator &= (uint16_t val) __attribute__((always_inline)) {
	  for(uint8_t i = 0;i<16;i++){
		  if(bitRead((save_ddr & mask),i)){
			 if(!bitRead(val,i)){
			    GPIO_PinModeSet(port,i,gpioModeInput,0);
				bitClear(save_ddr,i);
 	         }
		  }
	  }
      return *this;
    }
  private:
    uint16_t save_ddr = 0;	
};

#if  _GPIO_PORT_A_PIN_MASK >0
extern	DDRemulation	DDRA;
#endif
#if  _GPIO_PORT_B_PIN_MASK >0
extern	DDRemulation	DDRB;
#endif
#if  _GPIO_PORT_C_PIN_MASK >0
extern  DDRemulation	DDRC;
#endif
#if  _GPIO_PORT_D_PIN_MASK >0
extern  DDRemulation	DDRD;
#endif
#if  _GPIO_PORT_E_PIN_MASK >0
extern  DDRemulation	DDRE;
#endif
#if  _GPIO_PORT_F_PIN_MASK >0
extern  DDRemulation	DDRF;
#endif
#if  _GPIO_PORT_G_PIN_MASK >0
extern  DDRemulation	DDRG;
#endif
#if  _GPIO_PORT_H_PIN_MASK >0
extern  DDRemulation	DDRH;
#endif
#if  _GPIO_PORT_I_PIN_MASK >0
extern  DDRemulation	DDRI;
#endif
#if  _GPIO_PORT_J_PIN_MASK >0
extern  DDRemulation	DDRJ;
#endif
#if  _GPIO_PORT_K_PIN_MASK >0
extern  DDRemulation	DDRK;
#endif

#endif //__cplusplus

#if  _GPIO_PORT_A_PIN_MASK >0
#define PORTA   GPIOA.DOUT
#define  PINA   GPIOA.DIN
#endif
#if  _GPIO_PORT_B_PIN_MASK >0
#define PORTB  GPIOB.DOUT.DOUT
#define  PINB   GPIOB.DIN
#endif
#if  _GPIO_PORT_C_PIN_MASK >0
#define PORTC   GPIOC.DOUT
#define  PINC   GPIOC.DIN
#endif
#if  _GPIO_PORT_D_PIN_MASK >0
#define PORTD   GPIOD.DOUT
#define  PIND   GPIOD.DIN
#endif
#if  _GPIO_PORT_E_PIN_MASK >0
#define PORTE  GPIOE.DOUT
#define  PINE  GPIOE.DIN
#endif
#if  _GPIO_PORT_F_PIN_MASK >0
#define PORTF  GPIOF.DOUT
#define  PINF  GPIOF.DIN
#endif
#if  _GPIO_PORT_G_PIN_MASK >0
#define PORTG  GPIOG.DOUT
#define  PING  GPIOG.DIN
#endif
#if  _GPIO_PORT_H_PIN_MASK >0
#define PORTH  GPIOH.DOUT
#define  PINH  GPIOH.DIN
#endif
#if  _GPIO_PORT_I_PIN_MASK >0
#define PORTI  GPIOI.DOUT
#define  PINI  GPIOI.DIN
#endif
#if  _GPIO_PORT_J_PIN_MASK >0
#define PORTJ  GPIOJ.DOUT
#define  PINJ  GPIOJ.DIN
#endif
#if  _GPIO_PORT_K_PIN_MASK >0
#define PORTK  GPIOK.DOUT
#define  PINK  GPIOK.DIN
#endif

#endif //__AVREMULATION_H__
