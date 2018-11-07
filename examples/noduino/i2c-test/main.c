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
#include "noduino.h"

static uint8_t twi_sda = 4, twi_scl = 5;
#define SCL_LOW()   (GPES = (1 << twi_scl))
#define SCL_HIGH()  (GPEC = (1 << twi_scl))

#define SDA_LOW()   (GPES = (1 << twi_sda))
#define SDA_HIGH()  (GPEC = (1 << twi_sda))

#define get_SCL()  ((GPI & (1 << twi_scl)) != 0)
#define get_SDA()  ((GPI & (1 << twi_sda)) != 0)

static int clock_stretch = 3 * 230;
 

void i2c_delay(void)
{
	int i;

	int dcount = 42;	// about 100KHz

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
	int reg;
	for (i = 0; i < dcount; i++)
		reg = GPI;
#pragma GCC diagnostic pop
}

void i2c_init()
{
	pinMode(5, INPUT_PULLUP);
	pinMode(4, INPUT_PULLUP);
}
 
void i2c_start(void)
{
	SDA_HIGH();             // i2c start bit sequence
	SCL_HIGH();

	if (get_SDA() == 0) {
		serial_printf("i2c write start sda read false\r\n");
		return;
	}

	i2c_delay();

	SDA_LOW();
	i2c_delay();

	SCL_LOW();
	i2c_delay();
}
 
void i2c_stop(void)
{
	SCL_LOW();
	SDA_LOW();             // i2c stop bit sequence
	i2c_delay();

	SCL_HIGH();
	i2c_delay();

	SDA_HIGH();
	i2c_delay();
}

bool i2c_read_bit()
{
	int i = 0;

	SCL_LOW();
	SDA_HIGH();
	i2c_delay();

	SCL_HIGH();

	while(get_SCL() == 0 && (i++) < clock_stretch);

	bool b = get_SDA();          

	i2c_delay();
	return b;
}
 
bool i2c_write_byte(uint8_t d)
{
	uint8_t x;
	bool b;
	int i = 0;

	for(x = 8; x; x--) {

		SCL_LOW();

		if(d & 0x80)
			SDA_HIGH();
		else
			SDA_LOW();

		i2c_delay();

		SCL_HIGH();

		while (get_SCL() == 0 && (i++) < clock_stretch);
		i2c_delay();

		d <<= 1;
	}

	b = !i2c_read_bit();

	return !b;		//ACK or NACK
}

void setup()
{
	serial_begin(115200);

	wifi_set_opmode(NULL_MODE);		//disable wifi

	i2c_init();
}

void loop()
{
	serial_printf("Testing i2c write...\r\n");

	i2c_start();
	i2c_write_byte(0x00);
	i2c_write_byte(0x12);
	i2c_write_byte(0x1e);
	i2c_write_byte(0x03);
	i2c_write_byte(0x02);
	i2c_write_byte(0x00);
	i2c_write_byte(0x00);
	i2c_write_byte(0x1A);
	i2c_stop();

	delay(5000);
}
