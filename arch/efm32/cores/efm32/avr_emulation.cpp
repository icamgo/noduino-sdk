/*
  EMF32ARDUINO avr_emulation.cpp
  
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

#include "Arduino.h"

#if  _GPIO_PORT_A_PIN_MASK >0
DDRemulation   DDRA(gpioPortA,_GPIO_PORT_A_PIN_MASK);
#endif
#if  _GPIO_PORT_B_PIN_MASK >0
DDRemulation DDRB(gpioPortB,_GPIO_PORT_B_PIN_MASK);
#endif
#if  _GPIO_PORT_C_PIN_MASK >0
DDRemulation DDRC(gpioPortC,_GPIO_PORT_C_PIN_MASK);
#endif
#if  _GPIO_PORT_D_PIN_MASK >0
DDRemulation  DDRD(gpioPortD,_GPIO_PORT_D_PIN_MASK);
#endif
#if  _GPIO_PORT_E_PIN_MASK >0
DDRemulation DDRE(gpioPortE,_GPIO_PORT_E_PIN_MASK);
#endif
#if  _GPIO_PORT_F_PIN_MASK >0
DDRemulation DDRF(gpioPortF,_GPIO_PORT_F_PIN_MASK);
#endif
#if  _GPIO_PORT_G_PIN_MASK >0
DDRemulation DDRG(gpioPortG,_GPIO_PORT_G_PIN_MASK);
#endif
#if  _GPIO_PORT_H_PIN_MASK >0
DDRemulation DDRH(gpioPortH,_GPIO_PORT_H_PIN_MASK);
#endif
#if  _GPIO_PORT_I_PIN_MASK >0
DDRemulation DDRI(gpioPortI,_GPIO_PORT_I_PIN_MASK);
#endif
#if  _GPIO_PORT_J_PIN_MASK >0
DDRemulation DDRJ(gpioPortJ,_GPIO_PORT_J_PIN_MASK);
#endif
#if  _GPIO_PORT_K_PIN_MASK >0
DDRemulation DDRK(gpioPortK,_GPIO_PORT_K_PIN_MASK);
#endif
