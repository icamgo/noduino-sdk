/*
 *  Copyright (c) 2017 - 2025 MaiKe Labs
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
#include "softspi.h"

static uint8_t _cs;
static uint8_t _sck;
static uint8_t _mosi;
static uint8_t _miso;

void spi_init(uint8_t cs, uint8_t sck, uint8_t mo, uint8_t mi)
{
	_cs = cs;
	_sck = sck;
	_mosi = mo;
	_miso = mi;

	pinMode(_cs, OUTPUT);
	pinMode(_sck, OUTPUT);
	pinMode(_mosi, OUTPUT);
	pinMode(_miso, INPUT);

	//_cs = 1;
	digitalWrite(_cs, HIGH);
	//_sck = 0;
	digitalWrite(_sck, LOW);
}

void spi_end()
{
	pinMode(_cs, INPUT);
	pinMode(_sck, INPUT);
	pinMode(_mosi, INPUT);
	pinMode(_miso, INPUT);
}

static uint8_t spi_transfer(uint8_t data)
{
	uint8_t i;
	for (i = 0; i < 8; i++) {
		digitalWrite(_mosi, (data & 0x80));
		data = (data << 1);
		//_sck = 1;
		digitalWrite(_sck, HIGH);
		data |= digitalRead(_miso);
		//_sck = 0;
		digitalWrite(_sck, LOW);
	}
	return (data);
}

static void chip_select()
{
	//_cs = 0;
	digitalWrite(_cs, LOW);
}

static void chip_deselect()
{
	//_cs = 1;
	digitalWrite(_cs, HIGH);
}
///////////////////////////////////////

/* following is independent function */
uint8_t spi_read_reg(uint8_t reg)
{
	uint8_t reg_val;
	chip_select();

	spi_transfer(reg);
	reg_val = spi_transfer(0);

	chip_deselect();
	return (reg_val);
}

uint8_t spi_write_reg(uint8_t reg, uint8_t val)
{
	uint8_t status;
	chip_select();

	status = spi_transfer(reg);
	spi_transfer(val);

	chip_deselect();
	return (status);
}

uint8_t spi_read_buf(uint8_t reg, uint8_t *pbuf, uint8_t len)
{
	uint8_t status, i;
	chip_select();

	status = spi_transfer(reg);
	for (i = 0; i < len; i++)
		pbuf[i] = spi_transfer(0);

	chip_deselect();

	return (status);
}

uint8_t spi_write_buf(uint8_t reg, uint8_t *pbuf, uint8_t len)
{
	uint8_t status, i;
	chip_select();
	status = spi_transfer(reg);
	for (i = 0; i < len; i++)
		spi_transfer(*pbuf++);

	chip_deselect();

	return (status);
}
