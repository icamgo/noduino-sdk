/*
 *  Copyright (c) 2015 - 2025 MaiKe Labs
 *
 *  Example of BH1750 digital light library usage.
 *
 *  Using the default high resolution mode and then makes
 *   a light level reading every second.
 *
 *  Connect VCC of the BH1750 sensor to 3.3V (NOT 5.0V!)
 *  Connect GND to Ground
 *  Connect SCL to i2c clock - GPIO4
 *  Connect SDA to i2c data  - GPIO5
 *  Connect ADD-NC or GND
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
#include "bh1750.h"

irom void setup()
{
	serial_begin(115200);

	bh1750_begin(0x5C);		// the ADDR pin of BH1750 is HIGH, address is 0x5C

	serial_printf("BH1750 digital light sensor Example\r\n");
}

void loop()
{
	uint16_t lux = bh1750_readLightLevel();
	serial_printf("Current Environment Light is: %d lx\r\n", lux);
	delay(2000);
}
