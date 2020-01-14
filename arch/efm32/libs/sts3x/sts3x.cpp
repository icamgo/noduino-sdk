/*
 *  Copyright (c) 2018 - 2028 MaiKe Labs
 *
 *  Library for STS3x digital Temperature sensor
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
#include "sts3x.h"
#include "softi2c.h"

#define STS3X_1MS			(F_CPU/1000)		/* 14MHz, 1Tick = 1/14us, 14000tick = 1000us */
#define	sts3x_delay(x)		i2c_delay(x*STS3X_1MS)

#define DATA_LEN			3

static uint8_t crc8(const uint8_t *data, uint8_t len)
{
	// adapted from STS21 sample code from
	// http://www.sensirion.com/en/products/humidity-temperature/download-center/

	uint8_t crc = 0xff;
	uint8_t i;
	for (i = 0; i < len; ++i) {

		crc ^= data[i];

		for (uint8_t bit = 8; bit > 0; --bit) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x31;
			} else {
				crc = (crc << 1);
			}
		}
	}
	return crc;
}

uint8_t sts3x_clear_status()
{
	int ret;

	//Request the user register
	wire_beginTransmission(STS3X_ADDR_LOW);
	wire_write(CLEAR_STATUS >> 8);
	wire_write(CLEAR_STATUS & 0xff);
	ret = wire_endTransmission();

	// delay 11ms for 11bit resolution
	sts3x_delay(11);
	return ret;
}

uint8_t sts3x_reset(void)
{
	uint8_t ret;

	wire_beginTransmission(STS3X_ADDR_LOW);
	wire_write(SOFT_RESET >> 8);
	wire_write(SOFT_RESET & 0xff);
	ret = wire_endTransmission();

	sts3x_delay(2);
	return ret;
}

static uint16_t sts3x_read_sensor(uint16_t cmd)
{
	uint8_t data[DATA_LEN];
	int ret;

	wire_beginTransmission(STS3X_ADDR_LOW);
	wire_write(cmd >> 8);
	wire_write(cmd & 0xFF);
	ret = wire_endTransmission();

	// delay 11ms for 11bit resolution
	sts3x_delay(11);

	wire_requestFrom(STS3X_ADDR_LOW, 3);

	sts3x_delay(1);

	if (wire_available() < 3) {
		INFO("SHT2x: wire request(read) timeout!");
		return 1;
	}

	for (int i = 0; i < DATA_LEN; ++i) {
		data[i] = wire_read();
	}

	if (crc8(data, 2) != data[2]) {
		INFO("STS3X: crc error!");
		return 2;
	}

#ifdef DEBUG
	Serial.print(data[0], HEX);
	Serial.print(" ");

	Serial.print(data[1], HEX);
	Serial.print(" ");

	Serial.println(data[2], HEX);
#endif

	return ((data[0] << 8) | data[1]);
}

/*
 * Reset value:
 *
 *   High Alert set:   0x0133
 *   High Alert clear: 0x012D
 *   Low Alert set:    0x0066
 *   Low Alert clear:  0x0069
*/
void sts3x_show_alert()
{
	uint16_t has, hac, las, lac;

	has = sts3x_read_sensor(R_HIGH_ALERT_SET);
	hac = sts3x_read_sensor(R_HIGH_ALERT_CLR);

	las = sts3x_read_sensor(R_LOW_ALERT_SET);
	lac = sts3x_read_sensor(R_LOW_ALERT_CLR);

	Serial.print("High Alert set: ");
	Serial.print(has, HEX);
	Serial.print(" / ");

	Serial.println(-45.0 + 175.0*has*128 / 65535.0, 2);
	

	Serial.print("High Alert clear: ");
	Serial.print(hac, HEX);
	Serial.print(" / ");
	Serial.println(-45.0 + 175.0*hac*128 / 65535.0, 2);

	Serial.print("Low Alert set: ");
	Serial.print(las, HEX);
	Serial.print(" / ");
	Serial.println(-45.0 + 175.0*las*128 / 65535.0, 2);
	

	Serial.print("Low Alert clear: ");
	Serial.print(lac, HEX);
	Serial.print(" / ");
	Serial.println(-45.0 + 175.0*lac*128 / 65535.0, 2);
}

/*
 * Gets the current temperature from the sensor.
 * @return float - The temperature in Deg C
 */
float sts3x_get_temp(void)
{
	uint16_t sd = 2;
	int cnt = 0;
	float ret = -273.0;

	//sts3x_reset();
	sts3x_delay(2);

	while ((sd == 1 || sd == 2) && cnt <= 3) {
		sd = sts3x_read_sensor(READ_HIGH_RES);
		cnt++;
	}

	ret = (-45.0 + 175.0 * (float)sd / 65535.0);

	if (ret < -40.0)
		return -273.0;
	else if (ret > 120.0)
		return 300.0;
	else
		return ret;
}

uint8_t sts3x_init(uint8_t scl, uint8_t sda)
{
	wire_begin(scl, sda);

	return 0;
}
