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
#include "Arduino.h"

static int clock_stretch = 3 * 230;

#define I2C_DELAY			4		/* us delay */
#define I2C_MAXWAIT			5000

#define TWI_SDA		11		/* PIN14_PD7 */
#define TWI_SCL		16		/* PIN21_PF2 */

#define get_SDA()		digitalRead(TWI_SDA)
#define get_SCL()		digitalRead(TWI_SCL)

#if 1
#define SDA_LOW()		set_low(TWI_SDA)
#define SDA_HIGH() 		set_high(TWI_SDA)
#define SCL_LOW()		set_low(TWI_SCL)
#define SCL_HIGH()		set_high(TWI_SCL)

static void set_high(uint8_t pin)
{
	noInterrupts();
	pinMode(pin, INPUT_PULLUP);
	interrupts();
}

static void set_low(uint8_t pin)
{
	noInterrupts();
	digitalWrite(pin, LOW);
	pinMode(pin, OUTPUT);
	interrupts();
}
#else
#define SDA_LOW()		digitalWrite(TWI_SDA, LOW)
#define SDA_HIGH() 		digitalWrite(TWI_SDA, HIGH)
#define SCL_LOW()		digitalWrite(TWI_SCL, LOW)
#define SCL_HIGH()		digitalWrite(TWI_SCL, HIGH)

//#define i2c_delay(x) do{for(int i=0;i<x;i++) {asm volatile("nop");}}while(0)
#define i2c_delay(x)		delayMicroseconds(x)
#endif

#define i2c_delay() do{for(int i=0;i<42;i++) {asm volatile("nop");}}while(0)

//////////////////////////////////////////////////////////////

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
		Serial.println("i2c write start sda read false");
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
	Serial.setRouteLoc(1);
	Serial.begin(115200);

	i2c_init();
}

void loop()
{
	Serial.println("Testing i2c write...");

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
