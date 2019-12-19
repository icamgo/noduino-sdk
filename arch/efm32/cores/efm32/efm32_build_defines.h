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


#ifndef EFM32_BUILD_DEFINES_H
#define EFM32_BUILD_DEFINES_H

#define  ARDUINO_EXTEND 1

#if __has_include("HAL_Conf.h")
# include "HAL_Conf.h"
#endif

#if __has_include("configs/HAL_Conf.h")
# include "configs/HAL_Conf.h"
#endif

/*EFM/STM32 Compatibility definition */
#ifdef DEBUG_EFM
#  ifndef USE_FULL_ASSERT
#    define USE_FULL_ASSERT
#  endif
#endif
#ifdef USE_FULL_ASSERT
#  ifndef DEBUG_EFM
#    define DEBUG_EFM
#  endif
#endif

//default defines,  overriden by HAL_Conf.h  in path sketch or path valiants/valiant/configs/
/***************  HAL_Conf default here ******************/

//OS
#ifndef  FREERTOS
# define FREERTOS  0
#endif

#ifndef  UCOSII	/*running with ucosii*/
# define UCOSII      0
#endif

#ifndef BOOTLOADER	     /*check & go addr if avalible */
# define BOOTLOADER       0
#endif

#ifndef  USE_BITBAND
#define  USE_BITBAND     1
#endif

#ifndef USE_CORECALLBACK
#define USE_CORECALLBACK 0
#endif

#if 0
#ifndef USE_AVREMULATION
#define USE_AVREMULATION 1
#endif

#ifndef USE_TIMER0_PWM
# define USE_TIMER0_PWM 1
#endif

#ifndef USE_TIMER1_PWM
# define USE_TIMER1_PWM 1
#endif

#ifndef USE_TIMER2_PWM
# define USE_TIMER2_PWM 1
#endif

#ifndef USE_TIMER3_PWM
# define USE_TIMER3_PWM 1
#endif

//devices
//USB
#ifndef USE_USBSERIAL
# ifdef MENU_USB_SERIAL
#   define USE_USBSERIAL 1
#   define USE_USB 1
# endif
#endif

#ifndef USE_USBDMSC
# ifdef MENU_USB_MASS_STORAGE
#   define USE_USBDMSC  1
#   define USE_USB 1
# endif
#endif

#ifndef USE_USBDDFU
# ifdef MENU_USB_DFU
#   define USE_USBDDFU  1
#   define USE_USB 1
# endif
#endif

#ifndef USE_USBDCOMPOSITE
# ifdef MENU_USB_IAD
#	define USE_USBDCOMPOSITE  1
#   define USE_USB 1
# endif
#endif

#ifndef USE_USBDCONF
#  define USE_USBDCONF  1
#endif
#endif

//USART
#if 0
#ifndef USE_USART0
#define USE_USART0  1
#endif
#ifndef USE_USART1
#define USE_USART1  1
#endif
#ifndef USE_USART2
#define USE_USART2  1
#endif
#ifndef USE_USART3
#define USE_USART3  1
#endif
#ifndef USE_USART4
#define USE_USART4  1
#endif
#ifndef USE_USART5
#define USE_USART5  1
#endif
#ifndef USE_UART0
#define USE_UART0   1
#endif
#ifndef USE_UART1
#define USE_UART1   1
#endif
#endif

#ifndef USE_LEUART0
#define USE_LEUART0 1
#endif

#if 0
#ifndef USE_LEUART1
#define USE_LEUART1 1
#endif
#endif

#if 0
//SPI
#ifndef USE_SPI1
#define USE_SPI1 1
#endif

#ifndef USE_SPI2
#define USE_SPI2 1
#endif

#ifndef USE_SPI3
#define USE_SPI3 1
#endif

#ifndef USE_SPI4
#define USE_SPI4 1
#endif

#ifndef USE_SPI5
#define USE_SPI5 1
#endif

#ifndef USE_SPI6
#define USE_SPI6 1
#endif

//I2C
#ifndef USE_I2C1
#define USE_I2C1 1
#endif

#ifndef USE_I2C2
#define USE_I2C2 1
#endif

#ifndef USE_I2C3
#define USE_I2C3 1
#endif

#ifndef USE_I2C4
#define USE_I2C4 1
#endif
#endif

/***************  HAL_Conf default end  ******************/


#if __IGNORE

#elif defined(EFM32G222F128)||defined(EFM32G222F64)||defined(EFM32G222F32)

  #define EFM32G  1  
  #define CMSIS_STARTUP_C	"SiliconLabs/EFM32G/Source/GCC/startup_efm32g.c"
  #define CMSIS_SYSTEM_C	"SiliconLabs/EFM32G/Source/system_efm32g.c"
  #define CHIP_PIN_LIST PIN(A,0),PIN(A,1),PIN(A,2),PIN(A,8),PIN(A,9),PIN(A,10),PIN(B,7),PIN(B,8),PIN(B,11),PIN(B,13),PIN(B,14),PIN(C,0),PIN(C,1),PIN(C,2),PIN(C,3),PIN(C,4),PIN(C,8),PIN(C,9),PIN(C,10),PIN(C,11),PIN(C,13),PIN(C,14),PIN(C,15),PIN(D,4),PIN(D,5),PIN(D,6),PIN(D,7),PIN(E,10),PIN(E,11),PIN(E,12),PIN(E,13),PIN(F,0),PIN(F,1),PIN(F,2),PIN(F,3),PIN(F,4),PIN(F,5)

#elif defined(EFM32TG110F32)||defined(EFM32TG110F16)

  #define EFM32TG  1  
  #define CMSIS_STARTUP_C   "SiliconLabs/EFM32TG/Source/GCC/startup_efm32tg.c"
  #define CMSIS_SYSTEM_C    "SiliconLabs/EFM32TG/Source/system_efm32tg.c"
  #define CHIP_PIN_LIST PIN(A,0),PIN(B,7),PIN(B,8),PIN(B,11),PIN(B,13),PIN(B,14),PIN(C,0),PIN(C,1),PIN(C,14),PIN(C,15),PIN(D,6),PIN(D,7),PIN(E,12),PIN(E,13),PIN(F,0),PIN(F,1),PIN(F,2)

#elif defined(EFM32ZG110F32)

  #define EFM32ZG  1  
  #define CMSIS_STARTUP_C   "SiliconLabs/EFM32ZG/Source/GCC/startup_efm32zg.c"
  #define CMSIS_SYSTEM_C    "SiliconLabs/EFM32ZG/Source/system_efm32zg.c"
  #define CHIP_PIN_LIST PIN(A,0),PIN(B,7),PIN(B,8),PIN(B,11),PIN(B,13),PIN(B,14),PIN(C,0),PIN(C,1),PIN(C,14),PIN(C,15),PIN(D,6),PIN(D,7),PIN(E,12),PIN(E,13),PIN(F,0),PIN(F,1),PIN(F,2)

#elif defined(EFM32HG110F64)

  #define EFM32HG  1  
  #define CMSIS_STARTUP_C   "SiliconLabs/EFM32HG/Source/GCC/startup_efm32hg.c"
  #define CMSIS_SYSTEM_C    "SiliconLabs/EFM32HG/Source/system_efm32hg.c"
  #define CHIP_PIN_LIST PIN(A,0),PIN(B,7),PIN(B,8),PIN(B,11),PIN(B,13),PIN(B,14),PIN(C,0),PIN(C,1),PIN(C,14),PIN(C,15),PIN(D,6),PIN(D,7),PIN(E,12),PIN(E,13),PIN(F,0),PIN(F,1),PIN(F,2)

#else 
  #error UNKNOWN CHIP 
#endif

#endif
