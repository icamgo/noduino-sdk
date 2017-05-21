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
#ifndef __SPI__
#define __SPI__

void spi_init();
uint8_t spi_read_reg(uint8_t reg);
uint8_t spi_write_reg(uint8_t reg, uint8_t val);
uint8_t spi_read_buf(uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t spi_write_buf(uint8_t reg, uint8_t *buf, uint8_t len);

#endif
