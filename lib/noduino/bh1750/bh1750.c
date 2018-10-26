/*
 * This is a library for the BH1750FVI Digital Light Sensor
 * breakout board.
 * 
 * The board uses I2C for communication. 2 pins are required to
 * interface to the device.
 *
 * Connect SCL to i2c clock - GPIO4 (D8 in Noduino Falcon board)
 * Connect SDA to i2c data  - GPIO5 (D9 in Noduino Falcon board)
 *
 * Written by Christopher Laws, March, 2013.
 * 
 * Ported to Noduino SDK project by Jack Tan <jiankemeng@gmail.com>
 *
 */

#include "bh1750.h"

uint8_t bh1750_addr;

irom void bh1750_begin(uint8_t addr)
{
	uint8_t mode = BH1750_CONTINUOUS_HIGH_RES_MODE;

	bh1750_addr = addr;

	wire_begin();
	bh1750_setMode(mode);
}

irom void bh1750_setMode(uint8_t mode)
{
	switch (mode) {
	case BH1750_CONTINUOUS_HIGH_RES_MODE:
	case BH1750_CONTINUOUS_HIGH_RES_MODE_2:
	case BH1750_CONTINUOUS_LOW_RES_MODE:
	case BH1750_ONE_TIME_HIGH_RES_MODE:
	case BH1750_ONE_TIME_HIGH_RES_MODE_2:
	case BH1750_ONE_TIME_LOW_RES_MODE:
		// apply a valid mode change
		bh1750_write8(mode);
		delay(10);
		break;
	default:
		// Invalid measurement mode
#if BH1750_DEBUG == 1
		serial_printf("Invalid measurement mode");
#endif
		break;
	}
}

irom uint16_t bh1750_readLightLevel()
{
	uint16_t level;

	wire_beginTransmission(bh1750_addr);
	wire_requestFrom(bh1750_addr, 2);
	level = wire_read();
	level <<= 8;
	level |= wire_read();
	wire_endTransmission();

#if BH1750_DEBUG == 1
	serial_printf("Raw Light level: %d\r\n", level);
#endif

	level = level / 1.2;	// convert to lux

#if BH1750_DEBUG == 1
	serial_printf("Light level: %d\r\n", level);
#endif
	return level;
}

irom void bh1750_write8(uint8_t d)
{
	wire_beginTransmission(bh1750_addr);
	wire_write(d);
	wire_endTransmission();
}
