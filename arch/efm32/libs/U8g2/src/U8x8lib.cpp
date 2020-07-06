/*

  U8x8lib.cpp
  
  Arduino specific low level functions


  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)

  Copyright (c) 2016, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  

*/


#include "U8x8lib.h"
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif 

#ifdef U8X8_HAVE_HW_I2C
//#include <Wire.h>
#endif

size_t U8X8::write(uint8_t v) 
{
  if ( v == '\n' )
  {
    uint8_t dy = u8x8_pgm_read(u8x8.font+3);		/* new 2019 format */
    ty+=dy;
    tx=0;
  }
  else
  {
    uint8_t dx = u8x8_pgm_read(u8x8.font+2);		/* new 2019 format */
    u8x8_DrawGlyph(&u8x8, tx, ty, v);

    tx+=dx;
  }
  return 1;
}

/*=== ARDUINO GPIO & DELAY ===*/
#ifdef U8X8_USE_PINS
extern "C" uint8_t u8x8_gpio_and_delay_arduino(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
  uint8_t i;
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
    
      for( i = 0; i < U8X8_PIN_CNT; i++ )
	if ( u8x8->pins[i] != U8X8_PIN_NONE )
	{
	  if ( i < U8X8_PIN_OUTPUT_CNT )
	  {
	    pinMode(u8x8->pins[i], OUTPUT);
	  }
	  else
	  {
#ifdef INPUT_PULLUP
	    pinMode(u8x8->pins[i], INPUT_PULLUP);
#else
	    pinMode(u8x8->pins[i], OUTPUT);
	    digitalWrite(u8x8->pins[i], 1);
#endif 
	  }
	}
	  
      break;

#ifndef __AVR__	
    /* this case is not compiled for any AVR, because AVR uC are so slow */
    /* that this delay does not matter */
    case U8X8_MSG_DELAY_NANO:
      delayMicroseconds(arg_int==0?0:1);
      break;
#endif
    
    case U8X8_MSG_DELAY_10MICRO:
      /* not used at the moment */
      break;
    
    case U8X8_MSG_DELAY_100NANO:
      /* not used at the moment */
      break;
   
    case U8X8_MSG_DELAY_MILLI:
      delay(arg_int);
      break;
    case U8X8_MSG_DELAY_I2C:
      /* arg_int is 1 or 4: 100KHz (5us) or 400KHz (1.25us) */
      delayMicroseconds(arg_int<=2?5:2);
      break;
    case U8X8_MSG_GPIO_I2C_CLOCK:
    case U8X8_MSG_GPIO_I2C_DATA:
      if ( arg_int == 0 )
      {
	pinMode(u8x8_GetPinValue(u8x8, msg), OUTPUT);
	digitalWrite(u8x8_GetPinValue(u8x8, msg), 0);
      }
      else
      {
#ifdef INPUT_PULLUP
	pinMode(u8x8_GetPinValue(u8x8, msg), INPUT_PULLUP);
#else
	pinMode(u8x8_GetPinValue(u8x8, msg), OUTPUT);
	digitalWrite(u8x8_GetPinValue(u8x8, msg), 1);
#endif 
      }
      break;
    default:
      if ( msg >= U8X8_MSG_GPIO(0) )
      {
	i = u8x8_GetPinValue(u8x8, msg);
	if ( i != U8X8_PIN_NONE )
	{
	  if ( u8x8_GetPinIndex(u8x8, msg) < U8X8_PIN_OUTPUT_CNT )
	  {
	    digitalWrite(i, arg_int);
	  }
	  else
	  {
	    if ( u8x8_GetPinIndex(u8x8, msg) == U8X8_PIN_OUTPUT_CNT )
	    {
	      // call yield() for the first pin only, u8x8 will always request all the pins, so this should be ok
	      yield();
	    }
	    u8x8_SetGPIOResult(u8x8, digitalRead(i) == 0 ? 0 : 1);
	  }
	}
	break;
      }
      
      return 0;
  }
  return 1;
}
#endif // U8X8_USE_PINS


/*=============================================*/
/* fast SW I2C for AVR uC */
#if 0 

/* the following static vars are recalculated in U8X8_MSG_BYTE_START_TRANSFER */
/* so, it should be possible to use multiple displays with different pins */

static volatile uint8_t *arduino_i2c_clock_port;

static uint8_t arduino_i2c_clock_mask;
static uint8_t arduino_i2c_clock_n_mask;

static volatile uint8_t *arduino_i2c_data_port;
static uint8_t arduino_i2c_data_mask;
static uint8_t arduino_i2c_data_n_mask;

/*
  software i2c,
  ignores ACK response (which is anyway not provided by some displays)
  also does not allow reading from the device
*/
static void i2c_delay(u8x8_t *u8x8) U8X8_NOINLINE;
static void i2c_delay(u8x8_t *u8x8)
{
  //u8x8_gpio_Delay(u8x8, U8X8_MSG_DELAY_10MICRO, u8x8->display_info->i2c_bus_clock_100kHz);
  u8x8_gpio_Delay(u8x8, U8X8_MSG_DELAY_I2C, u8x8->display_info->i2c_bus_clock_100kHz);
}

static void i2c_init(u8x8_t *u8x8)
{
  *arduino_i2c_clock_port |= arduino_i2c_clock_mask;
  *arduino_i2c_data_port |= arduino_i2c_data_mask;
  i2c_delay(u8x8);
}

/* actually, the scl line is not observed, so this procedure does not return a value */

static void i2c_read_scl_and_delay(u8x8_t *u8x8)
{
  /* set as input (line will be high) */
  *arduino_i2c_clock_port |= arduino_i2c_clock_mask;

  i2c_delay(u8x8);
}

static void i2c_clear_scl(u8x8_t *u8x8)
{
  *arduino_i2c_clock_port &= arduino_i2c_clock_n_mask;
}

static void i2c_read_sda(u8x8_t *u8x8)
{
  /* set as input (line will be high) */
  *arduino_i2c_data_port |= arduino_i2c_data_mask;
}

static void i2c_clear_sda(u8x8_t *u8x8)
{
  /* set open collector and drive low */
  *arduino_i2c_data_port &= arduino_i2c_data_n_mask;
}

static void i2c_start(u8x8_t *u8x8)
{
  if ( u8x8->i2c_started != 0 )
  {
    /* if already started: do restart */
    i2c_read_sda(u8x8);     /* SDA = 1 */
    i2c_delay(u8x8);
    i2c_read_scl_and_delay(u8x8);
  }
  i2c_read_sda(u8x8);
  /* send the start condition, both lines go from 1 to 0 */
  i2c_clear_sda(u8x8);
  i2c_delay(u8x8);
  i2c_clear_scl(u8x8);
  u8x8->i2c_started = 1;
}


static void i2c_stop(u8x8_t *u8x8)
{
  /* set SDA to 0 */
  i2c_clear_sda(u8x8);  
  i2c_delay(u8x8);
 
  /* now release all lines */
  i2c_read_scl_and_delay(u8x8);
 
  /* set SDA to 1 */
  i2c_read_sda(u8x8);
  i2c_delay(u8x8);
  u8x8->i2c_started = 0;
}

static void i2c_write_bit(u8x8_t *u8x8, uint8_t val)
{
  if (val)
    i2c_read_sda(u8x8);
  else
    i2c_clear_sda(u8x8);
 
  i2c_delay(u8x8);
  i2c_read_scl_and_delay(u8x8);
  i2c_clear_scl(u8x8);
}

static void i2c_read_bit(u8x8_t *u8x8)
{
  //uint8_t val;
  /* do not drive SDA */
  i2c_read_sda(u8x8);
  i2c_delay(u8x8);
  i2c_read_scl_and_delay(u8x8);
  i2c_read_sda(u8x8);
  i2c_delay(u8x8);
  i2c_clear_scl(u8x8);
  //return val;
}

static void i2c_write_byte(u8x8_t *u8x8, uint8_t b)
{
  i2c_write_bit(u8x8, b & 128);
  i2c_write_bit(u8x8, b & 64);
  i2c_write_bit(u8x8, b & 32);
  i2c_write_bit(u8x8, b & 16);
  i2c_write_bit(u8x8, b & 8);
  i2c_write_bit(u8x8, b & 4);
  i2c_write_bit(u8x8, b & 2);
  i2c_write_bit(u8x8, b & 1);
    
  /* read ack from client */
  /* 0: ack was given by client */
  /* 1: nothing happend during ack cycle */  
  i2c_read_bit(u8x8);
}


extern "C" uint8_t u8x8_byte_arduino_sw_i2c(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
  uint8_t *data;
 
  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
      data = (uint8_t *)arg_ptr;
      
      while( arg_int > 0 )
      {
	i2c_write_byte(u8x8, *data);
	data++;
	arg_int--;
      }
      
      break;
      
    case U8X8_MSG_BYTE_INIT:
      pinMode(u8x8->pins[U8X8_PIN_I2C_CLOCK], OUTPUT);
      digitalWrite(u8x8->pins[U8X8_PIN_I2C_CLOCK], 1);

      pinMode(u8x8->pins[U8X8_PIN_I2C_DATA], OUTPUT);
      digitalWrite(u8x8->pins[U8X8_PIN_I2C_DATA], 1);

      i2c_init(u8x8);
      break;
    case U8X8_MSG_BYTE_SET_DC:
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
    
      /* there is no consistency checking for u8x8->pins[U8X8_PIN_I2C_CLOCK] */
    
      arduino_i2c_clock_port = portOutputRegister(digitalPinToPort(u8x8->pins[U8X8_PIN_I2C_CLOCK]));
      arduino_i2c_clock_mask = digitalPinToBitMask(u8x8->pins[U8X8_PIN_I2C_CLOCK]);
      arduino_i2c_clock_n_mask = ~arduino_i2c_clock_mask;
    
      /* there is no consistency checking for u8x8->pins[U8X8_PIN_I2C_DATA] */

      arduino_i2c_data_port = portOutputRegister(digitalPinToPort(u8x8->pins[U8X8_PIN_I2C_DATA]));
      arduino_i2c_data_mask = digitalPinToBitMask(u8x8->pins[U8X8_PIN_I2C_DATA]);
      arduino_i2c_data_n_mask = ~arduino_i2c_data_mask;

      i2c_start(u8x8);
      i2c_write_byte(u8x8, u8x8_GetI2CAddress(u8x8));
      
      break;
    case U8X8_MSG_BYTE_END_TRANSFER:
      i2c_stop(u8x8);
      break;
    default:
      return 0;
  }
  return 1;
  
}

#else

/* not AVR architecture, fallback */
extern "C" uint8_t u8x8_byte_arduino_sw_i2c(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
    return u8x8_byte_sw_i2c(u8x8, msg,arg_int, arg_ptr);
}
#endif

/*=== EFM32 HARDWARE I2C ===*/

#ifndef USE_SOFTI2C
static uint8_t I2C_ADDR = 0x78;

static void i2c_init(uint8_t addr)
{
	I2C_Init_TypeDef init = I2C_INIT_DEFAULT;

	init.enable = true;
	init.master = true;
	init.refFreq = 0;

#if F_CPU == 21000000L
	init.clhr = i2cClockHLRFast;
	init.freq = I2C_FREQ_FASTPLUS_MAX;
#elif F_CPU == 14000000L
	init.clhr = i2cClockHLRAsymetric;
	init.freq = I2C_FREQ_FAST_MAX;
#else
	init.clhr = i2cClockHLRStandard;
	init.freq = I2C_FREQ_STANDARD_MAX;
#endif

	I2C_ADDR = addr;

	/* Enabling clock to the I2C, GPIO*/
	CMU_ClockEnable(cmuClock_I2C0, true);

	/* PE12 - SDA, PE13 - SCL */
	GPIO_PinModeSet(gpioPortE, GPIO_PIN_12, gpioModeWiredAndPullUpFilter, 1);
	GPIO_PinModeSet(gpioPortE, GPIO_PIN_13, gpioModeWiredAndPullUpFilter, 1);

	I2C0->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN;
	I2C0->ROUTE = (I2C0->ROUTE & (~_I2C_ROUTE_LOCATION_MASK)) | I2C_ROUTE_LOCATION_LOC6;

	I2C_Init(I2C0, &init);
}

/*
 * I2C_FLAG_READ - data read into buf[0].data
 * I2C_FLAG_WRITE - data written from buf[0].data
 * I2C_FLAG_WRITE_READ - data written from buf[0].data and read into buf[1].data
 * I2C_FLAG_WRITE_WRITE - data written from buf[0].data and buf[1].data
 *
 */
#define TIME_OUTT 100

static I2C_TransferReturn_TypeDef i2c_write(uint8_t *buf, uint8_t len)
{
	uint32_t loop = TIME_OUTT;

	I2C_TransferReturn_TypeDef st;
	I2C_TransferSeq_TypeDef seq;

	seq.addr = I2C_ADDR;
	seq.flags = I2C_FLAG_WRITE;

	seq.buf[0].data = buf;
	seq.buf[0].len = len;

	st = I2C_TransferInit(I2C0, &seq);

	while ((st == i2cTransferInProgress) && loop > 0)
	{
		st = I2C_Transfer(I2C0);
		loop--;
	}

	return st;
}

#define WIRE_BUFFER_LEN		32
static uint8_t wire_rxBuffer[WIRE_BUFFER_LEN] = {0};
static uint8_t wire_rxBufferIndex = 0;
static uint8_t wire_rxBufferLength = 0;
static uint8_t wire_txAddress = 0;
static uint8_t wire_txBuffer[WIRE_BUFFER_LEN] = {0};
static uint8_t wire_txBufferIndex = 0;
static uint8_t wire_txBufferLength = 0;
static uint8_t wire_transmitting = 0;

static void wire_beginTransmission(uint8_t addr)
{
	wire_transmitting = 1;
	wire_txAddress = addr;
	wire_txBufferIndex = 0;
	wire_txBufferLength = 0;

	i2c_init(addr);
}

static size_t wire_write_b(uint8_t data)
{
	if(wire_transmitting){
		if(wire_txBufferLength >= WIRE_BUFFER_LEN){
			return 0;
		}
		wire_txBuffer[wire_txBufferIndex] = data;
		++wire_txBufferIndex;
		wire_txBufferLength = wire_txBufferIndex;
	}
	return 1;
}

static size_t wire_write(uint8_t *data, size_t len)
{
	if(wire_transmitting){

		// in master transmitter mode
		for(size_t i = 0; i < len; ++i){
			wire_write_b(data[i]);
		}
	}

	return len;
}

static uint8_t wire_endTransmission()
{
	int8_t ret;
	ret = i2c_write(wire_txBuffer, wire_txBufferLength);
	wire_txBufferIndex = 0;
	wire_txBufferLength = 0;
	wire_transmitting = 0;

	return ret;
}

extern "C" uint8_t u8x8_byte_arduino_hw_i2c(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
      wire_write((uint8_t *)arg_ptr, (int)arg_int);
      break;
    case U8X8_MSG_BYTE_INIT:
      //if ( u8x8->bus_clock == 0 ) 	/* issue 769 */
	  //u8x8->bus_clock = u8x8->display_info->i2c_bus_clock_100kHz * 100000UL;

	  i2c_init(u8x8_GetI2CAddress(u8x8));
	  //i2c_init(0x78);
      break;

    case U8X8_MSG_BYTE_SET_DC:
      break;

    case U8X8_MSG_BYTE_START_TRANSFER:
      wire_beginTransmission(u8x8_GetI2CAddress(u8x8));
      break;

    case U8X8_MSG_BYTE_END_TRANSFER:
      wire_endTransmission();
      break;
    default:
      return 0;
  }

  return 1;
}
#endif

#ifdef U8X8_USE_PINS
void u8x8_SetPin_SW_I2C(u8x8_t *u8x8, uint8_t clock, uint8_t data, uint8_t reset)
{
  u8x8_SetPin(u8x8, U8X8_PIN_I2C_CLOCK, clock);
  u8x8_SetPin(u8x8, U8X8_PIN_I2C_DATA, data);
  u8x8_SetPin(u8x8, U8X8_PIN_RESET, reset);
}

void u8x8_SetPin_HW_I2C(u8x8_t *u8x8, uint8_t reset, uint8_t clock, uint8_t data)
{
  u8x8_SetPin(u8x8, U8X8_PIN_RESET, reset);
  u8x8_SetPin(u8x8, U8X8_PIN_I2C_CLOCK, clock);
  u8x8_SetPin(u8x8, U8X8_PIN_I2C_DATA, data);
}
#endif // U8X8_USE_PINS
