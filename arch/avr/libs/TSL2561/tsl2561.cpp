/*
 *  Copyright (c) 2015 - 2025 MaiKe Labs
 *
 *  Library for TSL2561 light sensor 
 *
 *  This library is ported from adafruit Arduino TSL2561 project
 *  https://github.com/adafruit/TSL2561-Arduino-Library.git
 *
 *  Copyright (c) 2010, microBuilder SARL, Adafruit Industries
 *  All rights reserved.
 *
 *  @author   K. Townsend (microBuilder.eu / adafruit.com)
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

#include "tsl2561.h"

int8_t tsl2561_addr;
tsl2561IntegrationTime_t tsl2561_integration = TSL2561_INTEGRATIONTIME_13MS;
tsl2561Gain_t tsl2561_gain = TSL2561_GAIN_16X;
bool tsl2561_initialized = false;

bool tsl2561_begin(uint8_t addr)
{
	tsl2561_addr = addr;

	// Initialise I2C
	Wire.beginTransmission(tsl2561_addr);
	Wire.write(TSL2561_REGISTER_ID);
	Wire.endTransmission();
	Wire.requestFrom(tsl2561_addr, 1);
	int x = Wire.read();
	//Serial.print("0x"); Serial.println(x, HEX);
	if (x & 0x0A) {
		//Serial.println("Found TSL2561");
	} else {
		return false;
	}
	tsl2561_initialized = true;

	// Set default integration time and gain
	tsl2561_setTiming(tsl2561_integration);
	tsl2561_setGain(tsl2561_gain);
	// Note: by default, the device is in power down mode on bootup
	tsl2561_disable();

	return true;
}

void tsl2561_enable(void)
{
	// Enable the device by setting the control bit to 0x03
	tsl2561_write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL,
		       TSL2561_CONTROL_POWERON);
}

void tsl2561_disable(void)
{
	// Disable the device by setting the control bit to 0x03
	tsl2561_write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL,
		       TSL2561_CONTROL_POWEROFF);
}

void tsl2561_setGain(tsl2561Gain_t gain)
{
	tsl2561_enable();
	tsl2561_gain = gain;
	tsl2561_write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING,
		       tsl2561_integration | tsl2561_gain);
	tsl2561_disable();
}

void tsl2561_setTiming(tsl2561IntegrationTime_t integration)
{
	tsl2561_enable();
	tsl2561_integration = integration;
	tsl2561_write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING,
		       tsl2561_integration | tsl2561_gain);
	tsl2561_disable();
}

uint32_t tsl2561_calculateLux(uint16_t ch0, uint16_t ch1)
{
	unsigned long chScale;
	unsigned long channel1;
	unsigned long channel0;

	switch (tsl2561_integration) {
	case TSL2561_INTEGRATIONTIME_13MS:
		chScale = TSL2561_LUX_CHSCALE_TINT0;
		break;
	case TSL2561_INTEGRATIONTIME_101MS:
		chScale = TSL2561_LUX_CHSCALE_TINT1;
		break;
	default:		// No scaling ... integration time = 402ms
		chScale = (1 << TSL2561_LUX_CHSCALE);
		break;
	}

	// Scale for gain (1x or 16x)
	if (!tsl2561_gain)
		chScale = chScale << 4;

	// scale the channel values
	channel0 = (ch0 * chScale) >> TSL2561_LUX_CHSCALE;
	channel1 = (ch1 * chScale) >> TSL2561_LUX_CHSCALE;

	// find the ratio of the channel values (Channel1/Channel0)
	unsigned long ratio1 = 0;
	if (channel0 != 0)
		ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE + 1)) / channel0;

	// round the ratio value
	unsigned long ratio = (ratio1 + 1) >> 1;

	unsigned int b, m;

#ifdef TSL2561_PACKAGE_CS
	if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1C)) {
		b = TSL2561_LUX_B1C;
		m = TSL2561_LUX_M1C;
	} else if (ratio <= TSL2561_LUX_K2C) {
		b = TSL2561_LUX_B2C;
		m = TSL2561_LUX_M2C;
	} else if (ratio <= TSL2561_LUX_K3C) {
		b = TSL2561_LUX_B3C;
		m = TSL2561_LUX_M3C;
	} else if (ratio <= TSL2561_LUX_K4C) {
		b = TSL2561_LUX_B4C;
		m = TSL2561_LUX_M4C;
	} else if (ratio <= TSL2561_LUX_K5C) {
		b = TSL2561_LUX_B5C;
		m = TSL2561_LUX_M5C;
	} else if (ratio <= TSL2561_LUX_K6C) {
		b = TSL2561_LUX_B6C;
		m = TSL2561_LUX_M6C;
	} else if (ratio <= TSL2561_LUX_K7C) {
		b = TSL2561_LUX_B7C;
		m = TSL2561_LUX_M7C;
	} else if (ratio > TSL2561_LUX_K8C) {
		b = TSL2561_LUX_B8C;
		m = TSL2561_LUX_M8C;
	}
#else
	if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1T)) {
		b = TSL2561_LUX_B1T;
		m = TSL2561_LUX_M1T;
	} else if (ratio <= TSL2561_LUX_K2T) {
		b = TSL2561_LUX_B2T;
		m = TSL2561_LUX_M2T;
	} else if (ratio <= TSL2561_LUX_K3T) {
		b = TSL2561_LUX_B3T;
		m = TSL2561_LUX_M3T;
	} else if (ratio <= TSL2561_LUX_K4T) {
		b = TSL2561_LUX_B4T;
		m = TSL2561_LUX_M4T;
	} else if (ratio <= TSL2561_LUX_K5T) {
		b = TSL2561_LUX_B5T;
		m = TSL2561_LUX_M5T;
	} else if (ratio <= TSL2561_LUX_K6T) {
		b = TSL2561_LUX_B6T;
		m = TSL2561_LUX_M6T;
	} else if (ratio <= TSL2561_LUX_K7T) {
		b = TSL2561_LUX_B7T;
		m = TSL2561_LUX_M7T;
	} else if (ratio > TSL2561_LUX_K8T) {
		b = TSL2561_LUX_B8T;
		m = TSL2561_LUX_M8T;
	}
#endif

	unsigned long temp;
	temp = ((channel0 * b) - (channel1 * m));

	// do not allow negative lux value
	if (temp < 0)
		temp = 0;

	// round lsb (2^(LUX_SCALE-1))
	temp += (1 << (TSL2561_LUX_LUXSCALE - 1));

	// strip off fractional portion
	uint32_t lux = temp >> TSL2561_LUX_LUXSCALE;

	// Signal I2C had no errors
	return lux;
}

uint32_t tsl2561_getFullLuminosity(void)
{
	// Enable the device by setting the control bit to 0x03
	tsl2561_enable();

	// Wait x ms for ADC to complete
	switch (tsl2561_integration) {
	case TSL2561_INTEGRATIONTIME_13MS:
		delay(14);
		break;
	case TSL2561_INTEGRATIONTIME_101MS:
		delay(102);
		break;
	default:
		delay(403);
		break;
	}

	uint32_t x;
	x = tsl2561_read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT |
			   TSL2561_REGISTER_CHAN1_LOW);
	x <<= 16;
	x |= tsl2561_read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT |
			    TSL2561_REGISTER_CHAN0_LOW);

	tsl2561_disable();

	return x;
}

uint16_t tsl2561_get_luminosity(uint8_t channel)
{

	uint32_t x = tsl2561_getFullLuminosity();

	if (channel == 0) {
		// Reads two byte value from channel 0 (visible + infrared)
		return (x & 0xFFFF);
	} else if (channel == 1) {
		// Reads two byte value from channel 1 (infrared)
		return (x >> 16);
	} else if (channel == 2) {
		// Reads all and subtracts out just the visible!
		return ((x & 0xFFFF) - (x >> 16));
	}
	// unknown channel!
	return 0;
}

uint16_t tsl2561_read16(uint8_t reg)
{
	uint16_t x;
	uint16_t t;

	Wire.beginTransmission(tsl2561_addr);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(tsl2561_addr, 2);
	t = Wire.read();
	x = Wire.read();
	x <<= 8;
	x |= t;
	return x;
}

void tsl2561_write8(uint8_t reg, uint8_t value)
{
	Wire.beginTransmission(tsl2561_addr);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}
