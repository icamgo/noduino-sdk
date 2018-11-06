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

#define	SCL_HIGH()	digitalWrite(5, 1)
#define	SCL_LOW()	digitalWrite(5, 0)

#define	SDA_HIGH()	digitalWrite(4, 1)
#define	SDA_LOW()	digitalWrite(4, 0)

#define get_SCL()	digitalRead(5)
#define	get_SDA()	digitalRead(4)
 
void i2c_delay(void)
{
	int i;

	int dcount = 19;	// about 100KHz

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
	i2c_delay();

	SCL_HIGH();
	i2c_delay();

	SDA_LOW();
	i2c_delay();

	SCL_LOW();
	i2c_delay();
}
 
void i2c_stop(void)
{
	SDA_LOW();             // i2c stop bit sequence
	i2c_delay();

	SCL_HIGH();
	i2c_delay();

	SDA_HIGH();
	i2c_delay();
}
 
uint8_t i2c_rx(bool ack)
{
	uint8_t x, d=0;

	SDA_HIGH(); 

	for(x=0; x<8; x++) {
		d <<= 1;
		do {
			SCL_HIGH();
		} while(get_SCL() == 0);	// wait for any SCL clock stretching

		i2c_delay();

		if(get_SDA())
			d |= 1;

		SCL_LOW();
	} 

	if(ack)
		SDA_LOW();
	else
		SDA_HIGH();

	SCL_HIGH();
	i2c_delay();             // send (N)ACK bit
	SCL_LOW();
	SDA_HIGH();

	return d;
}
 
bool i2c_tx(uint8_t d)
{
	uint8_t x;
	static bool b;

	for(x = 8; x; x--) {

		if(d & 0x80)
			SDA_HIGH();
		else
			SDA_LOW();

		SCL_HIGH();
		d <<= 1;
		SCL_LOW();
	}

	SDA_HIGH();
	SCL_HIGH();

	i2c_delay();

	b = get_SDA();          // possible ACK bit

	SCL_LOW();
	return b;
}

void setup()
{
	serial_begin(115200);

	wifi_set_opmode(NULL_MODE);		//disable wifi

	i2c_init();
}

void loop()
{
	serial_printf("Hello World!\r\n");

	i2c_start();
	i2c_tx(0x00);
	i2c_tx(0x28);
	i2c_tx(0x44);
	i2c_tx(0x28);
	i2c_tx(0x44);
	i2c_stop();

	delay(5000);
}
