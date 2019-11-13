/*
 * Copyright (c) 2016 by Vassilis Serasidis <info@serasidis.gr>
 * 
 * Variant definition library for Arduino EFM32.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 *
 */ 

#ifndef __VARIANT_EFM32ZG110__
#define __VARIANT_EFM32ZG110__

#ifndef LED_BUILTIN
#define BOARD_NR_LED		1
#define LED_BUILTIN PA0
#define LED_BUILTIN_MASK	0x01
#endif

#define A6	PD6
#define A7	PD7
#define Pin2AdcChannel(x) (((x) == A6)?6:(((x) == A7)?7:0xff))

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

typedef struct _Pin2PortMapArray
{
	GPIO_Port_TypeDef GPIOx_Port; 	/* defined in em_gpio.h */
	uint8_t	Pin_abstraction;	/* emu pin*/		
	unsigned int adc_channel;
	TIMER_TypeDef *pTimer;		//Timer1 to Timer4.
	unsigned int timerChannelLoc;	//Timer bit3~0 channel (0-2)	bit7~4 routeLoc(0-3).	
} Pin2PortMapArray ;

/* Pins table to be instanciated into variant.cpp */
extern const Pin2PortMapArray g_Pin2PortMapArray[] ;

#ifdef __cplusplus
}
#endif

#define SERIAL_PORT_MONITOR			Serial
#define SERIAL_PORT_USBVIRTUAL		SerialUSB
#define SERIAL_PORT_HARDWARE_OPEN	Serial1
#define SERIAL_PORT_HARDWARE_OPEN1	Serial2

#define SERIAL_PORT_HARDWARE		Serial
#define SERIAL_PORT_HARDWARE1		Serial1
#define SERIAL_PORT_HARDWARE2		Serial2


#define SW_SDA			11		/* PIN14_PD7 */
#define SW_SCL			16		/* PIN21_PF2 */

#define SW_CS			2		/* PIN6 - PB8 - D2 */
#define SW_MOSI			6		/* PIN3 - PC0 - D6 */
#define SW_MISO			7		/* PIN4 - PC1 - D7 */
#define SW_SCK			1		/* PIN5 - PB7 - D1 */


#if 0
#define SCL		(PD7)
#define SDA		(PD6)

#define SPI_INTERFACES_COUNT 3

#define MOSI	(PC1)
#define MISO	(PC0)
#define SCK		(PB7)
#endif
 
#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

#ifdef __cplusplus
} //extern "C"{
#endif // __cplusplus

#endif
