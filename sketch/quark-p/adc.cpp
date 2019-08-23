/*
 *  Copyright (c) 2019 - 2029 MaiKe Labs
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
#include "adc.h"

#ifdef USE_MCP342x

// 0x68 is the default address for all MCP342x devices
uint8_t address = 0x68;
MCP342x adc = MCP342x(address);

uint8_t adc_gain = 8;

uint8_t adc_init(void)
{
	Wire.begin();

	MCP342x::generalCallReset();	// Reset devices
	delay(1);						// MC342x needs 300us to settle, wait 1ms

	// Check device present
	Wire.requestFrom(address, (uint8_t)1);

	if (!Wire.available()) {
		Serial.print("No device found at address ");
		Serial.println(address, HEX);
		return -1;
	}
	return 0;
}

long get_adc(void)
{
	long value = 0;
	MCP342x::Config status;

	// Initiate a conversion; convertAndRead() will wait until it can be read
	uint8_t err = adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot,
					MCP342x::resolution16, MCP342x::gain8,
					1000000, value, status);
	if (err) {
		Serial.print("Convert error: ");
		Serial.println(err);
		return -65535;
	} else {
		return value;
	}
}

float get_adc_uv()
{
	long data = get_adc();
	//uint32_t divi = (uint32_t ) 1 << 17;	// 17 = (18bit - 1)
	uint32_t divi = (uint32_t ) 1 << 15;	// 15 = (16bit - 1)

	// sizeof(long) = 4
	//Serial.print("sizof(long) = ");
	//Serial.println(sizeof(long));

	Serial.print("ADC = ");
	Serial.println(data);

	//Serial.print("mvDivisor = ");
	//Serial.println(divi);

    // voltage in millivolts
    float uv = data * 2048.0 * 1000.0 / divi / adc_gain;

	Serial.print("uv = ");
	Serial.println(uv);
    // uncomment line below to convert reading to microvolts
    return uv;
}
#endif
