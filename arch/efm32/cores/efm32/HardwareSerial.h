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

#ifndef _HARDWARESERIAL_H_
#define _HARDWARESERIAL_H_

#include "Stream.h"
#include "print.h"

#if !(defined(SERIAL_TX_BUFFER_SIZE) && defined(SERIAL_RX_BUFFER_SIZE))
 #define SERIAL_TX_BUFFER_SIZE 32
 #define SERIAL_RX_BUFFER_SIZE 256
#endif
 
#define SERIAL_8N1 0x06
#define SERIAL_8N2 0x0E

#define SERIAL_8E1 0x26
#define SERIAL_8E2 0x2E

#define SERIAL_8O1 0x36
#define SERIAL_8O2 0x3E

#ifndef SERIAL_BAUDRATE
#  define SERIAL_BAUDRATE 115200
#endif

#ifndef LESERIAL_BAUDRATE
# if (USE_LFBLFXO >0)||(USE_LFBLFRCO > 0)
#   if (SERIAL_BAUDRATE >9600)
#      define LESERIAL_BAUDRATE 9600   /*use LFXO or LFRCO lesart <=9600*/
#   else
#      define LESERIAL_BAUDRATE SERIAL_BAUDRATE
#   endif
# else
#  define LESERIAL_BAUDRATE SERIAL_BAUDRATE
# endif
#endif

#ifndef SPI_BAUDRATE
#define SPI_BAUDRATE            1000000
#endif
#ifndef SPI_PERCLK_FREQUENCY
#define SPI_PERCLK_FREQUENCY    F_CPU
#endif

typedef enum {
	USART_MODE_NONE    =   0,

	USART_TYPE         =0x01,
	LEUART_TYPE        =0x02,
	UART_TYPE          =0x04,
	SERIAL_MODE        =0x0f,  /*USART mode*/

	SPI_MAST_TYPE      =0x10,     
	SPI_SLAVE_TYPE     =0x20,
	SPI_MODE           =0x30, /*SPI mode*/
} USART_Mode_TypeDef;

typedef struct {
    USART_TypeDef *instance;
    USART_Mode_TypeDef mode = USART_MODE_NONE;
    uint8_t txBuffer[SERIAL_TX_BUFFER_SIZE];
#if (SERIAL_TX_BUFFER_SIZE >256)
    volatile uint16_t txStart=0;
    volatile uint16_t txEnd  =0;
#else
    volatile uint8_t  txStart=0;
    volatile uint8_t  txEnd  =0;
#endif
    uint8_t rxBuffer[SERIAL_RX_BUFFER_SIZE];
#if (SERIAL_TX_BUFFER_SIZE >256)
    volatile uint16_t rxStart=0;
    volatile uint16_t rxEnd  =0;
#else	
    volatile uint8_t  rxStart=0;
    volatile uint8_t  rxEnd  =0;
#endif
} USART_Buf_TypeDef;

class HardwareSerial : public Stream {
 public:
    HardwareSerial(USART_TypeDef *instance);
    /* Pin accessors */	
    void setRouteLoc(uint8_t route = 0);

    void begin(const uint32_t baud=115200, uint8_t config = SERIAL_8N1);
	void begin(USART_Mode_TypeDef mode = SERIAL_MODE);
    void end();
    virtual int available(void);
    int availableForWrite(void);
    virtual int peek(void);
    virtual int read(void);
    virtual void flush(void);
    virtual size_t write(uint8_t);
    using Print::write;
	operator bool() { return true; }
	
	USART_Buf_TypeDef *buf = NULL;
    USART_TypeDef *instance = NULL;

  private: 
    void initPort(void);
    void initSerialGpio(void);
    void initSpiGpio(USART_Mode_TypeDef  spiMode);
	uint32_t baud;
	uint8_t  config;
	uint32_t routeLoc = 0;
    uint32_t USART_ROUTE_LOCATION_LOCx = USART_ROUTE_LOCATION_LOC0;
  protected:
};

#if defined(USART0) && (USE_USART0 >0)
 	extern HardwareSerial SerialUSART0;
#endif
#if defined(USART1) && (USE_USART1 >0)
 	extern HardwareSerial SerialUSART1;
#endif
#if defined(USART2) && (USE_USART2 >0)
 	extern HardwareSerial SerialUSART2;
#endif
#if defined(USART3) && (USE_USART3 >0)
 	extern HardwareSerial Serial3;
#endif
#if defined(USART4) && (USE_USART4 >0)
 	extern HardwareSerial SerialUSART4;
#endif
#if defined(USART5) && (USE_USART5 >0)
 	extern HardwareSerial SerialUSART5;
#endif

#if defined(UART0) && (USE_UART0 >0)
 	extern HardwareSerial SerialUART0;
#endif
#if defined(UART1) && (USE_UART1 >0)
 	extern HardwareSerial SerialUART1;
#endif

#if defined(LEUART0) && (USE_LEUART0 >0)
    extern HardwareSerial SerialLEUART0;
#endif
#if defined(LEUART1) && (USE_LEUART1 >0)
    extern HardwareSerial SerialLEUART1;
#endif

#ifdef MENU_SERIAL
   #define Serial MENU_SERIAL
#endif	
#endif
