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

#include "Arduino.h"

#if defined(USART0)&& (USE_USART0 >0)
USART_Buf_TypeDef* USART0_buf = 0;
#endif
#if defined(USART1)&& (USE_USART1 >0)
USART_Buf_TypeDef* USART1_buf = 0;
#endif
#if defined(USART2)&& (USE_USART2 >0)
USART_Buf_TypeDef* USART2_buf = 0;
#endif
#if defined(USART3)&& (USE_USART3 >0)
USART_Buf_TypeDef* USART3_buf = 0;
#endif
#if defined(USART4)&& (USE_USART4 >0)
USART_Buf_TypeDef* USART4_buf = 0;
#endif
#if defined(USART5)&& (USE_USART5 >0)
USART_Buf_TypeDef* USART5_buf = 0;
#endif
#if  defined(UART0) && (USE_UART0 >0)
USART_Buf_TypeDef* UART0_buf = 0;
#endif
#if  defined(UART1) && (USE_UART1 >0)
USART_Buf_TypeDef* UART1_buf = 0;
#endif
#if defined(LEUART0)&& (USE_LEUART0 >0)
USART_Buf_TypeDef* LEUART0_buf = 0;
#endif
#if defined(LEUART1)&& (USE_LEUART1 >0)
USART_Buf_TypeDef* LEUART1_buf = 0;
#endif

HardwareSerial::HardwareSerial(USART_TypeDef *instance) {
  this->instance = instance;
}

void HardwareSerial::setRouteLoc(uint8_t route) {
      routeLoc = route;
      USART_ROUTE_LOCATION_LOCx = route<<8;
}

#if 0
void HardwareSerial::initSpiGpio(USART_Mode_TypeDef  spiMode) {
	
  USART_TypeDef *spi = this->instance;	
  /* Setting baudrate */
  spi->CLKDIV = 128 * (SPI_PERCLK_FREQUENCY / SPI_BAUDRATE - 2);

  /* Configure SPI */
  /* Using synchronous (SPI) mode*/

  spi->CTRL = USART_CTRL_SYNC;
  /* Clearing old transfers/receptions, and disabling interrupts */
  spi->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
  spi->IEN = 0;
  /* Enabling pins and setting location */
  spi->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN | USART_ROUTE_CSPEN | USART_ROUTE_LOCATION_LOCx;
	
    /* Set GPIO config to master */
  GPIO_Mode_TypeDef   gpioModeMosi = gpioModePushPull;
  GPIO_Mode_TypeDef   gpioModeMiso = gpioModeInput;
  GPIO_Mode_TypeDef   gpioModeCs   = gpioModePushPull;
  GPIO_Mode_TypeDef   gpioModeClk  = gpioModePushPull;

  if(spiMode == SPI_MAST_TYPE){
	gpioModeMosi = gpioModePushPull;
	gpioModeMiso = gpioModeInput;
	gpioModeCs   = gpioModePushPull;
	gpioModeClk  = gpioModePushPull;
  }else{
    /* Enabling TX and RX */
    spi->CMD = USART_CMD_TXEN | USART_CMD_RXEN;
  }
  /* Clear previous interrupts */
  spi->IFC = _USART_IFC_MASK;

  /* IO configuration */
#ifdef USART0
  if(spi == USART0){
	GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART0_TX_PORT(routeLoc), AF_USART0_TX_PIN(routeLoc), gpioModeMosi, 0);  /* TX/MOSI */
	GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART0_RX_PORT(routeLoc), AF_USART0_RX_PIN(routeLoc), gpioModeMiso, 0);  /* RX/MISO */
	GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART0_CS_PORT(routeLoc), AF_USART0_CS_PIN(routeLoc), gpioModeCs,   0);  /* CS */
	GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART0_CLK_PORT(routeLoc),AF_USART0_CLK_PIN(routeLoc),gpioModeClk,  0);  /* Clock */
  }
#endif
#ifdef USART1 
  if(spi == USART1){
	GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART1_TX_PORT(routeLoc), AF_USART0_TX_PIN(routeLoc), gpioModeMosi, 0);  /* TX/MOSI */
	GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART1_RX_PORT(routeLoc), AF_USART0_RX_PIN(routeLoc), gpioModeMiso, 0);  /* RX/MISO */
	GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART1_CS_PORT(routeLoc), AF_USART0_CS_PIN(routeLoc), gpioModeCs,   0);  /* CS */
	GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART1_CLK_PORT(routeLoc),AF_USART0_CLK_PIN(routeLoc),gpioModeClk,  0);  /* Clock */
  }
#endif
#ifdef USART2
  if(spi == USART2){
	GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART2_TX_PORT(routeLoc), AF_USART0_TX_PIN(routeLoc), gpioModeMosi, 0);  /* TX/MOSI */
	GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART2_RX_PORT(routeLoc), AF_USART0_RX_PIN(routeLoc), gpioModeMiso, 0);  /* RX/MISO */
	GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART2_CS_PORT(routeLoc), AF_USART0_CS_PIN(routeLoc), gpioModeCs,   0);  /* CS */
	GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART2_CLK_PORT(routeLoc),AF_USART0_CLK_PIN(routeLoc),gpioModeClk,  0);  /* Clock */
  };
#endif
#ifdef USART3
  if(spi == USART3){
	GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART3_TX_PORT(routeLoc), AF_USART0_TX_PIN(routeLoc), gpioModeMosi, 0);  /* TX/MOSI */
	GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART3_RX_PORT(routeLoc), AF_USART0_RX_PIN(routeLoc), gpioModeMiso, 0);  /* RX/MISO */
	GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART3_CS_PORT(routeLoc), AF_USART0_CS_PIN(routeLoc), gpioModeCs,   0);  /* CS */
	GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART3_CLK_PORT(routeLoc),AF_USART0_CLK_PIN(routeLoc),gpioModeClk,  0);  /* Clock */
  };
#endif
}
#endif

void HardwareSerial::initSerialGpio(void) {

	CMU_ClockEnable(cmuClock_GPIO, true);

#if  defined(USART0)&& (USE_USART0 >0)
  if (this->instance == USART0) {
	 
    /* To avoid false start, configure output as high */
    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART0_TX_PORT(this->routeLoc),
                    AF_USART0_TX_PIN(this->routeLoc),
                    gpioModePushPull, 1);

    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART0_RX_PORT(this->routeLoc),
                    AF_USART0_RX_PIN(this->routeLoc),
                    gpioModeInputPull, 0);
	buf->mode = USART_TYPE;
    }
#endif

#if   defined(USART1)&& (USE_USART1 >0)
  if (this->instance == USART1) {
    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART1_TX_PORT(this->routeLoc),
                    AF_USART1_TX_PIN(this->routeLoc),
                    gpioModePushPull, 1);

    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART1_RX_PORT(this->routeLoc),
                    AF_USART1_RX_PIN(this->routeLoc),
                    gpioModeInputPull, 0);
	buf->mode = USART_TYPE;
  }
#endif

#if   defined(USART2)&& (USE_USART2 >0)
  if (this->instance == USART2) {
    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART2_TX_PORT(this->routeLoc),
                    AF_USART2_TX_PIN(this->routeLoc),
                    gpioModePushPull, 1);

    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART2_RX_PORT(this->routeLoc),
                    AF_USART2_RX_PIN(this->routeLoc),
                    gpioModeInputPull, 0);
	buf->mode = USART_TYPE;
  }
#endif

#if   defined(USART3)&& (USE_USART3 >0)
  if (this->instance == USART3) {
    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART3_TX_PORT(this->routeLoc),
                    AF_USART3_TX_PIN(this->routeLoc),
                    gpioModePushPull, 1);

    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART3_RX_PORT(this->routeLoc),
                    AF_USART3_RX_PIN(this->routeLoc),
                    gpioModeInputPull, 0);
	buf->mode = USART_TYPE;
  }
#endif

#if   defined(USART4)&& (USE_USART4 >0)
  if (this->instance == USART4) {
    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART4_TX_PORT(this->routeLoc),
                    AF_USART4_TX_PIN(this->routeLoc),
                    gpioModePushPull, 1);

    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART4_RX_PORT(this->routeLoc),
                    AF_USART4_RX_PIN(this->routeLoc),
                    gpioModeInputPull, 0);
	buf->mode = USART_TYPE;
  }
#endif

#if   defined(USART5)&& (USE_USART5 >0)
  if (this->instance == USART5) {
    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART5_TX_PORT(this->routeLoc),
                    AF_USART5_TX_PIN(this->routeLoc),
                    gpioModePushPull, 1);

    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART5_RX_PORT(this->routeLoc),
                    AF_USART5_RX_PIN(this->routeLoc),
                    gpioModeInputPull, 0);
	buf->mode = USART_TYPE;
  }
#endif
#if   defined(UART0)&& (USE_UART0 >0)
  if (this->instance == UART0) {
    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_UART0_TX_PORT(this->routeLoc),
                    AF_UART0_TX_PIN(this->routeLoc),
                    gpioModePushPull, 1);

    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_UART0_RX_PORT(this->routeLoc),
                    AF_USART5_RX_PIN(this->routeLoc),
                    gpioModeInputPull, 0);
	buf->mode = UART_TYPE;
  }
#endif
#if   defined(UART1)&& (USE_UART1 >0)
  if (this->instance == UART1) {
    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_UART1_TX_PORT(this->routeLoc),
                    AF_UART1_TX_PIN(this->routeLoc),
                    gpioModePushPull, 1);

    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_UART1_RX_PORT(this->routeLoc),
                    AF_UART1_RX_PIN(this->routeLoc),
                    gpioModeInputPull, 0);
	buf->mode = UART_TYPE;
  }
#endif
#if defined(LEUART0)&& (USE_LEUART0 >0)
  if (this->instance == (USART_TypeDef *)LEUART0) {
    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_LEUART0_TX_PORT(this->routeLoc),
                    AF_LEUART0_TX_PIN(this->routeLoc),
                    gpioModePushPull, 1);

    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_LEUART0_RX_PORT(this->routeLoc),
                    AF_LEUART0_RX_PIN(this->routeLoc),
                    gpioModeInputPull, 0);
	buf->mode = LEUART_TYPE;
  }
#endif
#if   defined(LEUART1)&& (USE_LEUART1 >0)
  if (this->instance == (USART_TypeDef *)LEUART1) {
    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_LEUART1_TX_PORT(this->routeLoc),
                    AF_LEUART1_TX_PIN(this->routeLoc),
                    gpioModePushPull, 1);

    GPIO_PinModeSet((GPIO_Port_TypeDef)AF_LEUART1_RX_PORT(this->routeLoc),
                    AF_LEUART1_RX_PIN(this->routeLoc),
                    gpioModeInputPull, 0);
	buf->mode = LEUART_TYPE;
  }
#endif
}

void HardwareSerial::begin(USART_Mode_TypeDef spiMode){
  if(spiMode & SPI_MODE ){
     if (buf == NULL) {
       buf = (USART_Buf_TypeDef*)malloc(sizeof(USART_Buf_TypeDef));
       buf->instance = this->instance;
	   buf->mode = spiMode;
     }
     initSpiGpio(SPI_MODE);
     initPort();
  } else {
	 begin(SERIAL_BAUDRATE, SERIAL_8N1);
  }
}

void HardwareSerial::begin(const uint32_t baud,uint8_t config){
  this->baud = baud;
  this->config = config;
  if (buf == NULL) {
    buf = (USART_Buf_TypeDef*)malloc(sizeof(USART_Buf_TypeDef));
    buf->instance = this->instance;
  }
  initSerialGpio();  
  initPort();

  if (buf->mode == USART_TYPE) {
    USART_InitAsync_TypeDef initasync = USART_INITASYNC_DEFAULT;
    initasync.baudrate = baud;
    initasync.databits = usartDatabits8;

    if (config < 0x0f) {
      initasync.parity = usartNoParity;   /* No parity. */
    } else if (config < 0x2f) {
      initasync.parity = usartEvenParity; /* Even parity. */
    } else if (config < 0x3f) {
      initasync.parity =  usartOddParity; /* Odd parity. */
    }

    if ((config & 0x0f) == 0x0E) {
      initasync.stopbits = usartStopbits2; /* 1 stopbits. */
    } else {
      initasync.stopbits = usartStopbits1; /* 2 stopbits. */
    }

    initasync.oversampling = usartOVS16;
#if defined( USART_INPUT_RXPRS ) && defined( USART_CTRL_MVDIS )
    initasync.mvdis = 0;
    initasync.prsRxEnable = 0;
    initasync.prsRxCh = usartPrsRxCh0;
#endif
    USART_InitAsync(this->instance, &initasync);
    this->instance->IFC = _USART_IFC_MASK;
	this->instance->IEN = USART_IEN_RXDATAV;
    this->instance->ROUTE |=  USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_LOCATION_LOCx;
  }
  
  if (buf->mode == LEUART_TYPE) {

  }
}

void HardwareSerial::initPort(void) {
  
#if  defined(USART0)&& (USE_USART0 >0)
  if (this->instance == USART0) {
    USART0_buf = buf;
    USART_Reset(USART0);
    CMU_ClockEnable(cmuClock_USART0, true);
  }
#endif

#if   defined(USART1)&& (USE_USART1 >0)
  if (this->instance == USART1) {
    USART1_buf = buf;
    USART_Reset(USART1);
    CMU_ClockEnable(cmuClock_USART1, true);
  }
#endif

#if   defined(USART2)&& (USE_USART2 >0)
  if (this->instance == USART2) {
    USART2_buf = buf;
    USART_Reset(USART2);
    CMU_ClockEnable(cmuClock_USART2, true);
  }
#endif

#if   defined(USART3)&& (USE_USART3 >0)
  if (this->instance == USART3) {
    USART3_buf = buf;
    USART_Reset(USART3);
    CMU_ClockEnable(cmuClock_USART3, true);
  }
#endif

#if   defined(USART4)&& (USE_USART4 >0)
  if (this->instance == USART4) {
    USART4_buf = buf;
    USART_Reset(USART4);
    CMU_ClockEnable(cmuClock_USART4, true);
  }
#endif

#if   defined(USART5)&& (USE_USART5 >0)
  if (this->instance == USART5) {
    USART5_buf = buf;
    USART_Reset(USART5);
    CMU_ClockEnable(cmuClock_USART5, true);
  }
#endif
#if   defined(UART0)&& (USE_UART0 >0)
  if (this->instance == UART0) {
    UART0_buf = buf;
    UART_Reset(UART0);
    CMU_ClockEnable(cmuClock_UART0, true);
  }
#endif
#if   defined(UART1)&& (USE_UART1 >0)
  if (this->instance == UART1) {
    UART0_buf = buf;
    UART_Reset(UART1);
    CMU_ClockEnable(cmuClock_UART1, true);
  }
#endif

#if defined(LEUART0)&& (USE_LEUART0 >0)
  if (this->instance == (USART_TypeDef *)LEUART0) {

    LEUART0_buf = buf;
    LEUART_Reset(LEUART0);

    /* Enable clock for LEUART0 */
    CMU_ClockEnable(cmuClock_HFLE, true); // Necessary for accessing LE modules

	if (this->baud > 9600) {
		// Set CORELEDIV2 reference clock
		CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_CORELEDIV2);
	} else {
#if USE_LFBLFXO >0
		// use LFXO 32.768KHz as the clock source of LFB
		CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
#elif USE_LFBLFRCO > 0
		// use LFRCO 32.768KHz as the clock source of LFB
		CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFRCO);
#endif
	}

    CMU_ClockEnable(cmuClock_LEUART0, true);
	CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1); // Don't prescale LEUART clock

    LEUART_Init_TypeDef init = LEUART_INIT_DEFAULT;
    init.enable = leuartDisable;
	init.baudrate = this->baud;
    LEUART_Init(LEUART0, &init);
    LEUART0->IEN = LEUART_IEN_RXDATAV;
    LEUART0->ROUTE = USART_ROUTE_LOCATION_LOCx | LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN;
  }
#endif

#if   defined(LEUART1)&& (USE_LEUART1 >0)
  if (this->instance == (USART_TypeDef *)LEUART1) {
    LEUART1_buf = buf;
    LEUART_Reset(LEUART1);
    /* Enable clock for LEUART1 */
    CMU_ClockEnable(cmuClock_HFLE, true); // Necessary for accessing LE modules
#if USE_LFBLFXO >0
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO); // Set LFXO reference clock baud <=9600
#elif USE_LFBLFRCO > 0
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFRCO); // Set LFRCO reference clock baud <=9600
#else
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_CORELEDIV2); // Set CORELEDIV2 reference clock
#endif
    CMU_ClockEnable(cmuClock_LEUART1, true);
    LEUART_Init_TypeDef init = LEUART_INIT_DEFAULT;
    init.enable = leuartDisable;
	init.baudrate = this->baud;
    LEUART_Init(LEUART1, &init);
    LEUART1->IEN = LEUART_IEN_RXDATAV;
    LEUART1->ROUTE = USART_ROUTE_LOCATION_LOCx | LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN;
  }
#endif
  
#if defined(USART0)&& (USE_USART0 >0)
  if (this->instance == USART0) {
    NVIC_ClearPendingIRQ(USART0_RX_IRQn);
    NVIC_EnableIRQ(USART0_RX_IRQn);
    NVIC_ClearPendingIRQ(USART0_TX_IRQn);
    NVIC_EnableIRQ(USART0_TX_IRQn);
  }
#endif
#if defined(USART1)&& (USE_USART1 >0)
  if (this->instance == USART1) {
    NVIC_ClearPendingIRQ(USART1_RX_IRQn);
    NVIC_EnableIRQ(USART1_RX_IRQn);
    NVIC_ClearPendingIRQ(USART1_TX_IRQn);
    NVIC_EnableIRQ(USART1_TX_IRQn);
  }
#endif
#if defined(USART2)&& (USE_USART2 >0)
  if (this->instance == USART2) {
    NVIC_ClearPendingIRQ(USART2_RX_IRQn);
    NVIC_EnableIRQ(USART2_RX_IRQn);
    NVIC_ClearPendingIRQ(USART2_TX_IRQn);
    NVIC_EnableIRQ(USART2_TX_IRQn);
  }
#endif
#if defined(USART3)&& (USE_USART3 >0)
  if (this->instance == USART3) {
    NVIC_ClearPendingIRQ(USART3_RX_IRQn);
    NVIC_EnableIRQ(USART3_RX_IRQn);
    NVIC_ClearPendingIRQ(USART3_TX_IRQn);
    NVIC_EnableIRQ(USART3_TX_IRQn);
  }
#endif
#if defined(USART4)&& (USE_USART4 >0)
  if (this->instance == USART4) {
    NVIC_ClearPendingIRQ(USART4_RX_IRQn);
    NVIC_EnableIRQ(USART4_RX_IRQn);
    NVIC_ClearPendingIRQ(USART4_TX_IRQn);
    NVIC_EnableIRQ(USART4_TX_IRQn);
  }
#endif
#if defined(USART5)&& (USE_USART5 >0)
  if (this->instance == USART5) {
    NVIC_ClearPendingIRQ(USART5_RX_IRQn);
    NVIC_EnableIRQ(USART5_RX_IRQn);
    NVIC_ClearPendingIRQ(USART5_TX_IRQn);
    NVIC_EnableIRQ(USART5_TX_IRQn);
  }
#endif
#if defined(UART0)&& (USE_UART0 >0)
  if (this->instance == UART0) {
    NVIC_ClearPendingIRQ(UART0_RX_IRQn);
    NVIC_EnableIRQ(UART0_RX_IRQn);
    NVIC_ClearPendingIRQ(UART0_TX_IRQn);
    NVIC_EnableIRQ(UART0_TX_IRQn);
  }
#endif
#if defined(UART1)&& (USE_UART1 >0)
  if (this->instance == UART1) {
    NVIC_ClearPendingIRQ(UART1_RX_IRQn);
    NVIC_EnableIRQ(UART1_RX_IRQn);
    NVIC_ClearPendingIRQ(UART1_TX_IRQn);
    NVIC_EnableIRQ(UART1_TX_IRQn);
  }
#endif
#if defined(LEUART0)&& (USE_LEUART0 >0)
  if (this->instance == (USART_TypeDef *)LEUART0) {
    NVIC_ClearPendingIRQ(LEUART0_IRQn);
    NVIC_EnableIRQ(LEUART0_IRQn);
    LEUART0->IEN = LEUART_IEN_RXDATAV;
    LEUART_Enable(LEUART0, leuartEnable);

  }
#endif
#if defined(LEUART1)&& (USE_LEUART1 >0)
  if (this->instance == (USART_TypeDef *)LEUART1) {
    NVIC_ClearPendingIRQ(LEUART1_IRQn);
    NVIC_EnableIRQ(LEUART1_IRQn);
    LEUART1->IEN = LEUART_IEN_RXDATAV;
    LEUART_Enable(LEUART1, leuartEnable);
  }
#endif
}

void HardwareSerial::end(void) {}

int HardwareSerial::available(void)
{
  return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + buf->rxEnd - buf->rxStart)) % SERIAL_RX_BUFFER_SIZE;
}

int HardwareSerial::availableForWrite(void)
{
  if (buf->txEnd >= buf->txStart) return SERIAL_TX_BUFFER_SIZE - 1 - (buf->txEnd - buf->txStart); /*SERIAL_TX_BUFFER_SIZE-1 ~ 0*/
  return buf->txStart - buf->txEnd - 1;
}

int HardwareSerial::peek(void)
{
  if (available()) {
    return buf->rxBuffer[buf->rxStart];
  } else {
    return -1;
  }
}

void HardwareSerial::flush(void) {
  while (buf->txEnd != buf-> txStart);
}

int HardwareSerial::read(void) {
  if (available()) {
    uint8_t rtn =  buf->rxBuffer[buf->rxStart++];
    buf->rxStart %= SERIAL_RX_BUFFER_SIZE;
    return rtn;
  } else {
    return -1;
  }
}

size_t HardwareSerial::write(unsigned char ch) {
#if DEBUG_EFM /*for debug unused interrupt*/
    if (buf->mode == LEUART_TYPE) {
      LEUART_Tx((LEUART_TypeDef *)buf->instance, ch);
    }
  #if defined(UART0)||defined(UART1)
    else if(buf->mode == UART_TYPE){
      UART_Tx(buf->instance, ch);
    }
  #endif
    else {
      USART_Tx(buf->instance, ch);
    }

#else
  while (availableForWrite() == 0);   /*wait*/
  buf->txBuffer[buf->txEnd++] = ch;
  buf->txEnd %= SERIAL_TX_BUFFER_SIZE;
  if (buf->mode == LEUART_TYPE) {
    LEUART_IntEnable((LEUART_TypeDef *)buf->instance, LEUART_IEN_TXC);
    LEUART_IntSet((LEUART_TypeDef *)buf->instance, LEUART_IFS_TXC);
  }
#if defined(UART0)||defined(UART1)
  else if(buf->mode == UART_TYPE){
    UART_IntEnable(buf->instance, UART_IEN_TXC);
    UART_IntSet(buf->instance, UART_IFS_TXC);
  }
#endif
  else {
    USART_IntEnable(buf->instance, USART_IEN_TXC);
    USART_IntSet(buf->instance, USART_IFS_TXC);
  }
#endif  
  return 1;
}

void USART_TXCallback(USART_Buf_TypeDef *interruptUART) {
  if (interruptUART->txStart != interruptUART->txEnd) {
    USART_Tx(interruptUART->instance, interruptUART->txBuffer[interruptUART->txStart++]);
    interruptUART->txStart %= SERIAL_TX_BUFFER_SIZE;
  }
}
void USART_RXCallback(USART_Buf_TypeDef *interruptUART) {
  if (((interruptUART->rxEnd + 1) % SERIAL_RX_BUFFER_SIZE) == interruptUART->txStart) return; /*over*/
  interruptUART->rxBuffer[interruptUART->rxEnd++] = USART_Rx(interruptUART->instance);
  interruptUART->rxEnd %= SERIAL_RX_BUFFER_SIZE;
}
#if defined(USART0) && (USE_USART0 >0)
void (*usart0_rxCallbBck)(USART_Buf_TypeDef *interruptUART) = USART_RXCallback;
void (*usart0_txCallbBck)(USART_Buf_TypeDef *interruptUART) = USART_TXCallback;
extern "C"
void USART0_RX_IRQHandler(void)
{
    uint32_t flags = USART_IntGet(USART0);
    USART_IntClear(USART0, flags);
    if (flags & USART_IF_RXDATAV) usart0_rxCallbBck(USART0_buf);
}
extern "C"
void USART0_TX_IRQHandler(void)
{
  uint32_t flags = USART_IntGet(USART0);
  USART_IntClear(USART0, flags);
  if (flags & USART_IF_TXC) usart0_txCallbBck(USART0_buf);
//  if (flags & USART_IF_TXC) USART_TXCallback(USART0_buf);
}
HardwareSerial Serial0(USART0);
#endif

#if defined(USART1) && (USE_USART1 >0)
void (*usart1_rxCallbBck)(USART_Buf_TypeDef *interruptUART) = USART_RXCallback;
void (*usart1_txCallbBck)(USART_Buf_TypeDef *interruptUART) = USART_TXCallback;
extern "C"
void USART1_RX_IRQHandler(void)
{
  uint32_t flags = USART_IntGet(USART1);
  USART_IntClear(USART1, flags);
  if (flags & USART_IF_RXDATAV) usart1_rxCallbBck(USART1_buf);
}
extern "C"
void USART1_TX_IRQHandler(void)
{
  uint32_t flags = USART_IntGet(USART1);
  USART_IntClear(USART1, flags);
  if (flags & USART_IF_TXC) usart1_txCallbBck(USART1_buf);
}
HardwareSerial SerialUSART1(USART1);
#endif

#if defined(USART2) && (USE_USART2 >0)
void (*usart2_rxCallbBck)(USART_Buf_TypeDef *interruptUART) = USART_RXCallback;
void (*usart2_txCallbBck)(USART_Buf_TypeDef *interruptUART) = USART_TXCallback;
extern "C"
void USART2_RX_IRQHandler(void)
{
  uint32_t flags = USART_IntGet(USART2);
  USART_IntClear(USART2, flags);
  if (flags & USART_IF_RXDATAV) usart2_rxCallbBck(USART2_buf);
}
extern "C"
void USART2_TX_IRQHandler(void)
{
  uint32_t flags = USART_IntGet(USART2);
  USART_IntClear(USART2, flags);
  if (flags & USART_IF_TXC) usart2_txCallbBck(USART2_buf);
}
HardwareSerial SerialUSART2(USART2);
#endif

#if defined(USART3) && (USE_USART3 >0)
void (*usart3_rxCallbBck)(USART_Buf_TypeDef *interruptUART) = USART_RXCallback;
void (*usart3_txCallbBck)(USART_Buf_TypeDef *interruptUART) = USART_TXCallback;
extern "C"
void USART3_RX_IRQHandler(void)
{
  uint32_t flags = USART_IntGet(USART3);
  USART_IntClear(USART3, flags);
  if (flags & USART_IF_RXDATAV) usart3_rxCallbBck(USART3_buf);
}
extern "C"
void USART3_TX_IRQHandler(void)
{
  uint32_t flags = USART_IntGet(USART3);
  USART_IntClear(USART3, flags);
  if (flags & USART_IF_TXC) usart3_txCallbBck(USART3_buf);
}
HardwareSerial SerialUSART3(USART3);
#endif

#if defined(USART4) && (USE_USART4 >0)
void (*usart4_rxCallbBck)(USART_Buf_TypeDef *interruptUART) = USART_RXCallback;
void (*usart4_txCallbBck)(USART_Buf_TypeDef *interruptUART) = USART_TXCallback;
extern "C"
void USART4_RX_IRQHandler(void)
{
  uint32_t flags = USART_IntGet(USART4);
  USART_IntClear(USART4, flags);
  if (flags & USART_IF_RXDATAV) usart4_rxCallbBck(USART4_buf);
}
extern "C"
void USART4_TX_IRQHandler(void)
{
  uint32_t flags = USART_IntGet(USART4);
  USART_IntClear(USART4, flags);
  if (flags & USART_IF_TXC) usart4_txCallbBck(USART4_buf);
}
HardwareSerial SerialUSART4(USART4);
#endif

#if defined(USART5) && (USE_USART5 >0)
void (*usart5_rxCallbBck)(USART_Buf_TypeDef *interruptUART) = USART_RXCallback;
void (*usart5_txCallbBck)(USART_Buf_TypeDef *interruptUART) = USART_TXCallback;
extern "C"
void USART5_RX_IRQHandler(void)
{
  uint32_t flags = USART_IntGet(USART5);
  USART_IntClear(USART5, flags);
  if (flags & USART_IF_RXDATAV) usart5_rxCallbBck(USART5_buf);
}
extern "C"
void USART5_TX_IRQHandler(void)
{
  uint32_t flags = USART_IntGet(USART5);
  USART_IntClear(USART5, flags);
  if (flags & USART_IF_TXC) usart5_txCallbBck(USART5_buf);
}
HardwareSerial SerialUSART5(USART5);
#endif

#if defined(UART0)||defined(UART1)
void UART_TXCallback(USART_Buf_TypeDef *interruptUART) {
  if (interruptUART->txStart != interruptUART->txEnd) {
    UART_Tx((UART_TypeDef *)interruptUART->instance, interruptUART->txBuffer[interruptUART->txStart++]);
    interruptUART->txStart %= SERIAL_TX_BUFFER_SIZE;
  }
}
void UART_RXCallback(USART_Buf_TypeDef *interruptUART) {
  if (((interruptUART->rxEnd + 1) % SERIAL_RX_BUFFER_SIZE) == interruptUART->txStart) return; /*over*/
  interruptUART->rxBuffer[interruptUART->rxEnd++] = USART_Rx((UART_TypeDef *)interruptUART->instance);
  interruptUART->rxEnd %= SERIAL_RX_BUFFER_SIZE;
}

#if defined(UART0) && (USE_UART0 >0)
void (*uart0_rxCallbBck)(USART_Buf_TypeDef *interruptUART) = UART_RXCallback;
void (*uart0_txCallbBck)(USART_Buf_TypeDef *interruptUART) = UART_TXCallback;
extern "C"
void UART0_RX_IRQHandler(void)
{
  uint32_t flags = UART_IntGet(UART0);
  UART_IntClear(UART0, flags);
  if (flags & UART_IF_RXDATAV) uart0_rxCallbBck(UART0_buf);
}
extern "C"
void UART0_TX_IRQHandler(void)
{
  uint32_t flags = UART_IntGet(UART0);
  UART_IntClear(UART0, flags);
  if (flags & UART_IF_TXC) uart0_txCallbBck(UART0_buf);
}
HardwareSerial SerialUART0((USART_TypeDef *)UART0);
#  endif

#  if defined(UART1) && (USE_UART1 >0)
void (*uart1_rxCallbBck)(USART_Buf_TypeDef *interruptUART) = UART_RXCallback;
void (*uart1_txCallbBck)(USART_Buf_TypeDef *interruptUART) = UART_TXCallback;
extern "C"
void UART1_RX_IRQHandler(void)
{
  uint32_t flags = UART_IntGet(UART1);
  UART_IntClear(UART1, flags);
  if (flags & UART_IF_RXDATAV) uart1_rxCallbBck(UART1_buf);
}
extern "C"
void UART1_TX_IRQHandler(void)
{
  uint32_t flags = UART_IntGet(UART1);
  UART_IntClear(UART1, flags);
  if (flags & UART_IF_TXC) uart1_txCallbBck(UART1_buf);
}
HardwareSerial SerialUART1((USART_TypeDef *)UART1);
#  endif
#endif

#if (defined(LEUART0)&&(USE_LEUART0 >0))||(defined(LEUART1)&&(USE_LEUART1 >0))
void LEUART_TXCallback(USART_Buf_TypeDef *interruptUART) {
  if (interruptUART->txStart != interruptUART->txEnd) {
    LEUART_Tx((LEUART_TypeDef *)interruptUART->instance, interruptUART->txBuffer[interruptUART->txStart++]);
    interruptUART->txStart %= SERIAL_TX_BUFFER_SIZE;
  }
}
void LEUART_RXCallback(USART_Buf_TypeDef *interruptUART) {
  if (((interruptUART->rxEnd + 1) % SERIAL_RX_BUFFER_SIZE) == interruptUART->txStart) return; /*over*/
  interruptUART->rxBuffer[interruptUART->rxEnd++] = LEUART_Rx((LEUART_TypeDef *)interruptUART->instance);
  interruptUART->rxEnd %= SERIAL_RX_BUFFER_SIZE;
}
#if defined(LEUART0) && (USE_LEUART0 >0)
void (*leuart0_rxCallbBck)(USART_Buf_TypeDef *interruptUART) = LEUART_RXCallback;
void (*leuart0_txCallbBck)(USART_Buf_TypeDef *interruptUART) = LEUART_TXCallback;

extern "C"
void LEUART0_IRQHandler(void)
{
  uint32_t flags = LEUART_IntGet(LEUART0);
  LEUART_IntClear(LEUART0, flags);
  if (flags & LEUART_IF_RXDATAV) leuart0_rxCallbBck(LEUART0_buf);
  if (flags & LEUART_IF_TXC)     leuart0_txCallbBck(LEUART0_buf);
}
HardwareSerial SerialLEUART0((USART_TypeDef *)LEUART0);
#  endif
#  if defined(LEUART1) && (USE_LEUART1 >0)
void (*leuart1_rxCallbBck)(USART_Buf_TypeDef *interruptUART) = LEUART_RXCallback;
void (*leuart1_txCallbBck)(USART_Buf_TypeDef *interruptUART) = LEUART_TXCallback;
extern "C"
void LEUART1_IRQHandler(void)
{
  uint32_t flags = LEUART_IntGet(LEUART1);
  LEUART_IntClear(LEUART1, flags);
  if (flags & LEUART_IF_RXDATAV) leuart1_rxCallbBck(LEUART1_buf);
  if (flags & LEUART_IF_TXC)     leuart1_txCallbBck(LEUART1_buf);
}
HardwareSerial SerialLEUART1((USART_TypeDef *)LEUART1);
#  endif
#endif
