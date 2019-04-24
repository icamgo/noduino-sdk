/*
 *  Copyright (c) 2015 - 2025 MaiKe Labs
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
*/
#include "twi.h"

uint8_t twi_dcount = 18;
static uint8_t twi_sda = 255, twi_scl = 255;

//Enable SDA(becomes output and since GPO is 0 for the pin, it will pull the line low)
#define SDA_LOW()   (GPES = (1 << twi_sda))
//Disable SDA(becomes input and since it has pullup it will go high)
#define SDA_HIGH()  (GPEC = (1 << twi_sda))
#define SDA_READ()  ((GPI & (1 << twi_sda)) != 0)
#define SCL_LOW()   (GPES = (1 << twi_scl))
#define SCL_HIGH()  (GPEC = (1 << twi_scl))
#define SCL_READ()  ((GPI & (1 << twi_scl)) != 0)

#ifndef FCPU80
#define FCPU80 80000000L
#endif

#if F_CPU == FCPU80
#define TWI_CLOCK_STRETCH 800
#else
#define TWI_CLOCK_STRETCH 1600
#endif

void irom twi_setClock(unsigned int freq)
{
#if F_CPU == FCPU80
	if (freq <= 100000)
		twi_dcount = 19;
	//about 100 KHz
	else if (freq <= 200000)
		twi_dcount = 8;
	//about 200 KHz
	else if (freq <= 300000)
		twi_dcount = 3;
	//about 300 KHz
	else if (freq <= 400000)
		twi_dcount = 1;
	//about 400 KHz
	else
		twi_dcount = 1;
	//about 400 KHz
#else
	if (freq <= 100000)
		twi_dcount = 32;
	//about 100 KHz
	else if (freq <= 200000)
		twi_dcount = 14;
	//about 200 KHz
	else if (freq <= 300000)
		twi_dcount = 8;
	//about 300 KHz
	else if (freq <= 400000)
		twi_dcount = 5;
	//about 400 KHz
	else if (freq <= 500000)
		twi_dcount = 3;
	//about 500 KHz
	else if (freq <= 600000)
		twi_dcount = 2;
	//about 600 KHz
	else
		twi_dcount = 1;
	//about 700 KHz
#endif
}

void irom twi_init(uint8_t pin_sda, uint8_t pin_scl)
{
	twi_sda = pin_sda;
	twi_scl = pin_scl;
	pinMode(twi_sda, INPUT_PULLUP);
	pinMode(twi_scl, INPUT_PULLUP);
	twi_setClock(100000);
}

void irom twi_stop(void)
{
	pinMode(twi_sda, INPUT);
	pinMode(twi_scl, INPUT);
}

static void irom twi_delay(uint8_t v)
{
	unsigned int i;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
	unsigned int reg;
	for (i = 0; i < v; i++)
		reg = GPI;
#pragma GCC diagnostic pop
}

static bool irom twi_write_start(void)
{
	SCL_HIGH();
	SDA_HIGH();
	if (SDA_READ() == 0)
		return false;
	twi_delay(twi_dcount);
	SDA_LOW();
	twi_delay(twi_dcount);
	return true;
}

static bool irom twi_write_stop(void)
{
	unsigned int i = 0;
	SCL_LOW();
	SDA_LOW();
	twi_delay(twi_dcount);
	SCL_HIGH();
	while (SCL_READ() == 0 && (i++) < TWI_CLOCK_STRETCH) ;
	//Clock stretching(up to 100u s)
	twi_delay(twi_dcount);
	SDA_HIGH();
	twi_delay(twi_dcount);

	return true;
}

static bool irom twi_write_bit(bool bit)
{
	unsigned int i = 0;
	SCL_LOW();
	if (bit)
		SDA_HIGH();
	else
		SDA_LOW();
	twi_delay(twi_dcount + 1);
	SCL_HIGH();
	while (SCL_READ() == 0 && (i++) < TWI_CLOCK_STRETCH) ;
	//Clock stretching(up to 100u s)
	twi_delay(twi_dcount);
	return true;
}

static bool irom twi_read_bit()
{
	unsigned int i = 0;
	SCL_LOW();
	SDA_HIGH();
	twi_delay(twi_dcount + 2);
	SCL_HIGH();
	while (SCL_READ() == 0 && (i++) < TWI_CLOCK_STRETCH) ;
	//Clock stretching(up to 100u s)
	bool bit = SDA_READ();
	twi_delay(twi_dcount);
	return bit;
}

static bool irom twi_write_byte(uint8_t byte)
{
	uint8_t bit;
	for (bit = 0; bit < 8; bit++) {
		twi_write_bit(byte & 0x80);
		byte <<= 1;
	}
	return !twi_read_bit();
	//NACK, ACK
}

static uint8_t irom twi_read_byte(bool nack)
{
	uint8_t byte = 0;
	uint8_t bit;
	for (bit = 0; bit < 8; bit++)
		byte = (byte << 1) | twi_read_bit();
	twi_write_bit(nack);
	return byte;
}

uint8_t irom 
twi_writeTo(uint8_t address, uint8_t *buf, unsigned int len,
	    uint8_t sendStop)
{
	unsigned int i;
	if (!twi_write_start())
		return 4;
	//line busy
	if (!twi_write_byte(((address << 1) | 0) & 0xFF))
		return 2;
	//received NACK on transmit of address
	for (i = 0; i < len; i++) {
		if (!twi_write_byte(buf[i]))
			return 3;
		//received NACK on transmit of data
	}
	if (sendStop)
		twi_write_stop();
	i = 0;
	while (SDA_READ() == 0 && (i++) < 10) {
		SCL_LOW();
		twi_delay(twi_dcount);
		SCL_HIGH();
		twi_delay(twi_dcount);
	}
	return 0;
}

uint8_t irom 
twi_readFrom(uint8_t address, uint8_t *buf, unsigned int len,
	     uint8_t sendStop)
{
	unsigned int i;
	if (!twi_write_start())
		return 4;
	//line busy
	if (!twi_write_byte(((address << 1) | 1) & 0xFF))
		return 2;
	//received NACK on transmit of address
	for (i = 0; i < (len - 1); i++)
		buf[i] = twi_read_byte(false);
	buf[len - 1] = twi_read_byte(true);
	if (sendStop)
		twi_write_stop();
	i = 0;
	while (SDA_READ() == 0 && (i++) < 10) {
		SCL_LOW();
		twi_delay(twi_dcount);
		SCL_HIGH();
		twi_delay(twi_dcount);
	}
	return 0;
}

//////////////////////////////////////////////////////
#define WIRE_BUFFER_LEN		32
static uint8_t wire_rxBuffer[WIRE_BUFFER_LEN] = {0};
static uint8_t wire_rxBufferIndex = 0;
static uint8_t wire_rxBufferLength = 0;
static uint8_t wire_txAddress = 0;
static uint8_t wire_txBuffer[WIRE_BUFFER_LEN] = {0};
static uint8_t wire_txBufferIndex = 0;
static uint8_t wire_txBufferLength = 0;
static uint8_t wire_transmitting = 0;

void irom wire_begin()
{
	twi_init(SDA, SCL);
}

void irom wire_setClock(uint32_t clk)
{
	twi_setClock(clk);
}

void irom wire_beginTransmission(uint8_t addr)
{
	wire_transmitting = 1;
	wire_txAddress = addr;
	wire_txBufferIndex = 0;
	wire_txBufferLength = 0;
}

size_t irom wire_write(uint8_t data)
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

uint8_t irom wire_endTransmission()
{
	int8_t ret;
	ret = twi_writeTo(wire_txAddress, wire_txBuffer, wire_txBufferLength, true);
	wire_txBufferIndex = 0;
	wire_txBufferLength = 0;
	wire_transmitting = 0;
	return ret;
}

uint8_t irom wire_requestFrom(uint8_t addr, size_t len)
{
	if(len > WIRE_BUFFER_LEN) {
		len = WIRE_BUFFER_LEN;
	}
	size_t read = (twi_readFrom(addr, wire_rxBuffer, len, true) == 0) ? len : 0;
	wire_rxBufferIndex = 0;
	wire_rxBufferLength = read;
	return read;
}

int irom wire_read()
{
	int value = -1;
	if(wire_rxBufferIndex < wire_rxBufferLength){
		value = wire_rxBuffer[wire_rxBufferIndex];
		++wire_rxBufferIndex;
	}
	return value;
}

int irom wire_available(void)
{
	int result = wire_rxBufferLength - wire_rxBufferIndex;

	return result;
}
