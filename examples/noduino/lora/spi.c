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
#include "spi.h"

void spi_init()
{
	pinMode(CS, OUTPUT);
	pinMode(SCK, OUTPUT);
	pinMode(MOSI, OUTPUT);
	pinMode(MISO, INPUT);

	//CS = 1;
	digitalWrite(CS, HIGH);
	//SCK = 0;
	digitalWrite(SCK, LOW);
}

static uint8_t spi_transfer(uint8_t data)
{
	uint8_t i;
	for (i = 0; i < 8; i++) {
		digitalWrite(MOSI, (data & 0x80));
		data = (data << 1);
		//SCK = 1;
		digitalWrite(SCK, HIGH);
		data |= digitalRead(MISO);
		//SCK = 0;
		digitalWrite(SCK, LOW);
	}
	return (data);
}

static void chip_select()
{
	//CS = 0;
	digitalWrite(CS, LOW);
}

static void chip_deselect()
{
	//CS = 1;
	digitalWrite(CS, HIGH);
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
