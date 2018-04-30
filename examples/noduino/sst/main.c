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
#include "sht2x.h"
#include "tsl2561.h"
#include "bmp180.h"

irom void setup_tsl2561()
{
	if (!tsl2561_begin(TSL2561_ADDR_LOW)) {
		serial_print("Could not found TSL2561 sensor!\r\n");
	}

	// You can change the gain on the fly, to adapt to brighter/dimmer
	// light situations
	//tsl2561_setGain(TSL2561_GAIN_0X);		// set no gain (for bright situtations)
	tsl2561_setGain(TSL2561_GAIN_16X);		// set 16x gain (for dim situations)

	// Changing the integration time gives you a longer time over which to sense light
	// longer timelines are slower, but are good in very low light situtations!
	tsl2561_setTiming(TSL2561_INTEGRATIONTIME_13MS);		// shortest integration time (bright light)
	//tsl2561_setTiming(TSL2561_INTEGRATIONTIME_101MS);		// medium integration time (medium light)
	//tsl2561_setTiming(TSL2561_INTEGRATIONTIME_402MS);		// longest integration time (dim light)
}

void do_tsl2561()
{
	// Simple data read example. Just read the infrared, fullspecrtrum diode
	// or 'visible' (difference between the two) channels.
	// This can take 13-402 milliseconds! Uncomment whichever of the following you want to read
	uint16_t x = tsl2561_getLuminosity(TSL2561_VISIBLE);
	//uint16_t x = tsl2561_getLuminosity(TSL2561_FULLSPECTRUM);
	//uint16_t x = tsl2561_getLuminosity(TSL2561_INFRARED);

	serial_printf("Luminosity:\t\t%d\r\n", x);

	// More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
	// That way you can do whatever math and comparisons you want!
	uint32_t lum = tsl2561_getFullLuminosity();
	uint16_t ir, full;
	ir = lum >> 16;
	full = lum & 0xFFFF;

	serial_printf("IR:\t\t\t%d\r\n", ir);
	serial_printf("Full:\t\t\t%d\r\n", full);
	serial_printf("Visible:\t\t%d\r\n", full - ir);
	serial_printf("Lux:\t\t\t%d\r\n", tsl2561_calculateLux(full, ir));
}

irom void setup_bmp180()
{
	if (!bmp180_begin()) {
		serial_print("Could not find a valid BMP180 sensor!\r\n");
	}
}

void do_bmp180()
{
	char obuf[16];

	serial_printf("Pressure:\t\t%d Pa\r\n", bmp180_readPressure());

	dtostrf(bmp180_readTemperature(), 16, 1, obuf);
	serial_printf("Temperature:\t%s C\r\n", obuf);
}

void do_sht2x()
{
	char t_buf[8];
	char h_buf[8];

	sht2x_reset();

	dtostrf(sht2x_GetTemperature(), 5, 1, t_buf),
	dtostrf(sht2x_GetHumidity(), 5, 1, h_buf);
	serial_printf("Temperature(C):\t\t\t%s\r\n", t_buf);
	serial_printf("Humidity(%):\t\t\t%s\r\n\r\n", h_buf);
}

void setup()
{
	serial_begin(115200);
	serial_printf("Start to test SHT2x sensor!\r\n");
	wire_begin();

	setup_tsl2561();
	setup_bmp180();
}

void loop()
{
	do_sht2x();
	do_bmp180();
	do_tsl2561();

	serial_printf("#####################################\r\n");

	delay(6000);
}
