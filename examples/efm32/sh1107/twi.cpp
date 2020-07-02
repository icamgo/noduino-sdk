/*
 * Copyright (c) 2015 - 2025 MaiKe Labs
 * 
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at your
 * option) any later version.
 * 
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "twi.h"

#define SDA_READ()		digitalRead(_sda)
#define SCL_READ()		digitalRead(_scl)

#if 1
#define SDA_LOW()		set_low(_sda)
#define SDA_HIGH() 		set_high(_sda)
#define SCL_LOW()		set_low(_scl)
#define SCL_HIGH()		set_high(_scl)
#else
#define SDA_LOW()		GPIO_PinModeSet(gpioPortE, GPIO_PIN_12, gpioModePushPull, false)
#define SDA_HIGH()		GPIO_PinModeSet(gpioPortE, GPIO_PIN_12, gpioModePushPull, true)
#define SCL_LOW()		GPIO_PinModeSet(gpioPortE, GPIO_PIN_13, gpioModePushPull, false)
#define SCL_HIGH()		GPIO_PinModeSet(gpioPortE, GPIO_PIN_13, gpioModePushPull, true)
#endif

static uint8_t _scl = SW_SCL;
static uint8_t _sda = SW_SDA;

static inline void set_high(uint8_t pin)
{
	noInterrupts();
	pinMode(pin, INPUT_PULLUP);
	interrupts();
}

static inline void set_low(uint8_t pin)
{
	noInterrupts();
	digitalWrite(pin, LOW);
	pinMode(pin, OUTPUT);
	interrupts();
}

//////////////////////////////////////////////////////////////

bool inline i2c_init(uint8_t scl, uint8_t sda)
{
	_scl = scl;
	_sda = sda;

	SDA_HIGH();
	SCL_HIGH();

	if (SDA_READ() == 0 || SCL_READ() == 0)
		return false;

	return true;
}

void inline i2c_deinit(void)
{
	SDA_HIGH();
	SCL_HIGH();
	pinMode(_sda, INPUT);
	pinMode(_scl, INPUT);
}

bool inline i2c_start(void)
{
	SCL_HIGH();
	SDA_HIGH();

	if (SDA_READ() == 0) {
		INFO("twi write start sda read false");
		return false;
	}

	i2c_delay(I2C_DELAY);
	SDA_LOW();
	i2c_delay(I2C_DELAY);
	return true;
}

bool inline i2c_stop(void)
{
	unsigned int i = 0;
	SCL_LOW();
	SDA_LOW();
	i2c_delay(I2C_DELAY);
	SCL_HIGH();
	
	while (SCL_READ() == 0 && (i++) < I2C_MAXWAIT);

	i2c_delay(I2C_DELAY);
	SDA_HIGH();
	i2c_delay(I2C_DELAY);

	return true;
}

static inline bool i2c_write_bit(bool bit)
{
	unsigned int i = 0;

	SCL_LOW();

	if (bit) {
		SDA_HIGH();
	} else {
		SDA_LOW();
	}

	i2c_delay(I2C_DELAY + 1);

	SCL_HIGH();

	while (SCL_READ() == 0 && (i++) < I2C_MAXWAIT);

	i2c_delay(I2C_DELAY);

	return true;
}

static inline bool i2c_read_bit()
{
	unsigned int i = 0;

	SCL_LOW();
	SDA_HIGH();
	i2c_delay(I2C_DELAY + 2);

	SCL_HIGH();

	while (SCL_READ() == 0 && (i++) < I2C_MAXWAIT);

	bool bit = SDA_READ();
	i2c_delay(I2C_DELAY);

	return bit;
}

bool i2c_write_byte(uint8_t byte)
{
	uint8_t bit;

	for (bit = 0; bit < 8; bit++) {
		i2c_write_bit(byte & 0x80);
		byte <<= 1;
	}

	return !i2c_read_bit();
}

uint8_t i2c_read_byte(bool nack)
{
	uint8_t byte = 0;
	uint8_t bit;

	for (bit = 0; bit < 8; bit++)
		byte = (byte << 1) | i2c_read_bit();

	i2c_write_bit(nack);

	return byte;
}

uint8_t 
i2c_writeTo(uint8_t addr, uint8_t *buf, unsigned int len,
	    uint8_t sendStop)
{
	unsigned int i;
	if (!i2c_start()) {
		INFO("I2C: bus busy");
		return 4;
	}
	if (!i2c_write_byte(((addr << 1) | 0) & 0xFF)) {
		if (sendStop)
			i2c_stop();
		INFO("I2C: received NACK on transmit of addr");
		return 2;
	}
	for (i = 0; i < len; i++) {
		if (!i2c_write_byte(buf[i])) {
			if (sendStop)
				i2c_stop();
			INFO("I2C: received NACK on transmit of data");
			return 3;
		}
	}
	if (sendStop)
		i2c_stop();
	i = 0;
	while (SDA_READ() == 0 && (i++) < 10) {
		SCL_LOW();
		i2c_delay(I2C_DELAY);
		SCL_HIGH();
		i2c_delay(I2C_DELAY);
	}
	return 0;
}

uint8_t 
i2c_readFrom(uint8_t addr, uint8_t *buf, unsigned int len,
	     uint8_t sendStop)
{
	unsigned int i;

	if (!i2c_start()) {
		INFO("I2C: bus busy");
		return 4;
	}

	if (!i2c_write_byte(((addr << 1) | 1) & 0xFF)) {

		if (sendStop)
			i2c_stop();

		INFO("I2C: received NACK on transmit of addr");
		return 2;
	}

	for (i = 0; i < (len - 1); i++)
		buf[i] = i2c_read_byte(false);

	buf[len - 1] = i2c_read_byte(true);

	if (sendStop)
		i2c_stop();

	i = 0;

	while (SDA_READ() == 0 && (i++) < 10) {
		SCL_LOW();
		i2c_delay(I2C_DELAY);
		SCL_HIGH();
		i2c_delay(I2C_DELAY);
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

void wire_begin(uint8_t scl, uint8_t sda)
{
	i2c_init(scl, sda);
}

void wire_end()
{
	i2c_deinit();
}

void wire_setClock(uint32_t clk)
{

}

void wire_beginTransmission(uint8_t addr)
{
	wire_transmitting = 1;
	wire_txAddress = addr;
	wire_txBufferIndex = 0;
	wire_txBufferLength = 0;
}

size_t wire_write(uint8_t data)
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

uint8_t wire_endTransmission()
{
	int8_t ret;
	ret = i2c_writeTo(wire_txAddress, wire_txBuffer, wire_txBufferLength, true);
	wire_txBufferIndex = 0;
	wire_txBufferLength = 0;
	wire_transmitting = 0;
	return ret;
}

uint8_t wire_requestFrom(uint8_t addr, size_t len)
{
	if(len > WIRE_BUFFER_LEN) {
		len = WIRE_BUFFER_LEN;
	}
	size_t read = (i2c_readFrom(addr, wire_rxBuffer, len, true) == 0) ? len : 0;
	wire_rxBufferIndex = 0;
	wire_rxBufferLength = read;
	return read;
}

int wire_read()
{
	int value = -1;
	if(wire_rxBufferIndex < wire_rxBufferLength){
		value = wire_rxBuffer[wire_rxBufferIndex];
		++wire_rxBufferIndex;
	}
	return value;
}

int wire_available(void)
{
	int result = wire_rxBufferLength - wire_rxBufferIndex;

	if (!result) {
		// yielding here will not make more data "available",
		// but it will prevent the system from going into WDT reset
		return 0;
	}

	return result;
}
