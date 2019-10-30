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

#ifndef EFM32_PIN_LIST_H
#define EFM32_PIN_LIST_H

#ifdef VARIANT_PIN_LIST
	#define PIN_LIST VARIANT_PIN_LIST
#else
	#define PIN_LIST CHIP_PIN_LIST
#endif

#ifdef __cplusplus

class __ConstPin {
  public:
    constexpr __ConstPin(const GPIO_Port_TypeDef GPIOx_Port, const  uint16_t pinMask, const uint8_t ucPin)
      : GPIOx_Port(GPIOx_Port), pinMask(pinMask), ucPin(ucPin) {};
    constexpr operator uint8_t() const {
      return ucPin;
    }
    const GPIO_Port_TypeDef GPIOx_Port;
    const uint16_t pinMask;
    const uint8_t ucPin;
};
#define PIN(a, b) __P##a##b
enum {
  PIN_LIST,
  NUM_DIGITAL_PINS,
};
#undef PIN

/*Compatible with stm32 arduino core*/
#define PIN(a, b) P##a##b(gpioPort ## a,GPIO_PIN_ ## b, __P##a##b)
constexpr __ConstPin PIN_LIST;
#undef PIN

/*Compatible with Nuvoton arduino core*/
#define PIN(a, b) P##a##_##b(gpioPort ## a,GPIO_PIN_ ## b, __P##a##b)
constexpr __ConstPin PIN_LIST;
#undef PIN

#define ARDUINOPIN_TypeDef  __ConstPin

#else  /*c mode*/

#define ARDUINOPIN_TypeDef  uint8_t

/*Compatible with stm32 arduino core*/
#define PIN(a, b) P##a##b
enum {
  PIN_LIST,
  NUM_DIGITAL_PINS,
};
#undef PIN

/*Compatible with Nuvoton arduino core*/
#define PIN(a, b) P##a##_##b
enum {
  PIN_LIST,
};
#undef PIN

#endif /*__cplusplus*/

#endif
