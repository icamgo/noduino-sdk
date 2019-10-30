/****************************************************************************
 * Copyright (c) 2016 by Vassilis Serasidis <info@serasidis.gr>
 * 
 * Variant definition library for Arduino EFM32.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 
 ****************************************************************************
 *     2016.9.18 huawei <huaweiwx@sina.com>                                *
 ****************************************************************************/


#ifndef _VARIANT_EMF32DIP_H_
#define _VARIANT_EMF32DIP_H_

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#ifndef LED_BUILTIN
 #define BOARD_NR_LED  1
 #define LED_BUILTIN PD6
 #define LED_BUILTIN_MASK	0x01
#endif

#define A4  PD4
#define A5  PD5
#define A6  PD6
#define A7  PD7

#define Pin2AdcChannel(x) (((x) == A4)?4:(((x) == A5)?5:(((x) == A6)?6:(((x) == A7)?7:0xff))))

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

typedef struct _Pin2PortMapArray
{
  	GPIO_Port_TypeDef GPIOx_Port; 	/* defined in em_gpio.h */
  	uint8_t  Pin_abstraction;	/* emu pin*/    
    unsigned int  adc_channel;
    TIMER_TypeDef *pTimer;   //Timer1 to Timer4.
    unsigned int  timerChannelLoc;  //Timer bit3~0 channel (0-2)  bit7~4 routeLoc(0-3).  
} Pin2PortMapArray ;

/* Pins table to be instanciated into variant.cpp */
extern const Pin2PortMapArray g_Pin2PortMapArray[] ;

#ifdef __cplusplus
}
#endif


/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_HARDWARE_OPEN   Serial1
#define SERIAL_PORT_HARDWARE_OPEN1  Serial2

#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE2       Serial2

#define SCL    (PA1)
#define SDA    (PA0)
//#define SCL    (PD7)
//#define SDA    (PD6)


#define SPI_INTERFACES_COUNT 3

#ifdef USE_SPI1
   /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
 #define BOARD_SPI1_MOSI_PIN   (PA7)   
 #define BOARD_SPI1_MISO_PIN   (PA6)   
 #define BOARD_SPI1_SCK_PIN    (PA5)   
#endif

#ifdef USE_SPI2
    /**SPI2 GPIO Configuration    
    PB10     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
 #define BOARD_SPI2_SCK_PIN     (PB10)   
 #define BOARD_SPI2_MISO_PIN    (PB14)   
 #define BOARD_SPI2_MOSI_PIN    (PB15)   
#endif

#ifdef USE_SPI3
    /**SPI3 GPIO Configuration    
    PB3     ------> SPI3_SCK
    PB4     ------> SPI3_MISO
    PB5     ------> SPI3_MOSI 
    */
 
// #define BOARD_SPI3_NSS_PIN		(PXX)
 #define BOARD_SPI3_SCK_PIN         (PB3)   
 #define BOARD_SPI3_MISO_PIN        (PB4)   
 #define BOARD_SPI3_MOSI_PIN        (PB5)   
#endif

#ifdef USE_I2C3
 #define  BOARD_NR_I2C 3U
#elif defined(USE_I2C2)
 #define  BOARD_NR_I2C 2U
#elif defined(USE_I2C1)
 #define  BOARD_NR_I2C 1U
#endif

#ifdef USE_SPI3
 #define  BOARD_NR_SPI 3U
#elif defined(USE_SPI2)
 #define  BOARD_NR_SPI 2U
#elif defined(USE_SPI1)
 #define  BOARD_NR_SPI 1U
#endif


#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

#ifdef __cplusplus
} //extern "C"{
#endif // __cplusplus

#endif
