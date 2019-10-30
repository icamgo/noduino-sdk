/*
  EMF32ARDUINO efm32pwm.h

  Copyright (c) 2018 huaweiwx@sina.com 2018.7.1

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

#ifndef _EFMM32_PWM_H_
#define _EFMM32_PWM_H_

#ifdef __cplusplus
extern "C"{
#endif

void analogWrite(uint8_t ucPin,uint32_t val);

#ifdef __cplusplus
}
#endif

#endif //_EFMM32_PWM_H_
