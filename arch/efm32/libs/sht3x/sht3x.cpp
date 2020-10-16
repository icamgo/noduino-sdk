/*
 *  Copyright (c) 2018 - 2028 MaiKe Labs
 *
 *  Library for SHT3x digital Temperature sensor
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
#include "sht3x.h"
#include "softi2c.h"

#define SHT3X_1MS			(F_CPU/1000)		/* 14MHz, 1Tick = 1/14us, 14000tick = 1000us */
#define	sht3x_delay(x)		i2c_delay(x*SHT3X_1MS)

#define DATA_LEN			6

static uint8_t crc8(const uint8_t *data, uint8_t len)
{
	// adapted from SHT21 sample code from
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

uint8_t sht3x_clear_status()
{
	int ret;

	//Request the user register
	wire_beginTransmission(SHT3X_ADDR);
	wire_write(CLEAR_STATUS >> 8);
	wire_write(CLEAR_STATUS & 0xff);
	ret = wire_endTransmission();

	// delay 11ms for 11bit resolution
	sht3x_delay(11);
	return ret;
}

uint8_t sht3x_reset(void)
{
	uint8_t ret;

	wire_beginTransmission(SHT3X_ADDR);
	wire_write(SOFT_RESET >> 8);
	wire_write(SOFT_RESET & 0xff);
	ret = wire_endTransmission();

	sht3x_delay(2);
	return ret;
}

static uint8_t sht3x_read_data(uint16_t cmd, uint8_t *data, size_t len)
{
	int ret;

	wire_beginTransmission(SHT3X_ADDR);
	wire_write(cmd >> 8);
	wire_write(cmd & 0xFF);
	ret = wire_endTransmission();

	// delay 11ms for 11bit resolution
	sht3x_delay(11);

	wire_requestFrom(SHT3X_ADDR, len);

	sht3x_delay(1);

	if (wire_available() < len) {
		INFO("SHT3x: wire request(read) timeout!");
		return 1;
	}

	for (int i = 0; i < len; ++i) {
		data[i] = wire_read();
	}

#ifdef DEBUG
	Serial.print(data[0], HEX);
	Serial.print(" ");

	Serial.print(data[1], HEX);
	Serial.print(" ");

	Serial.println(data[2], HEX);
#endif

	return 0; 
}

int sht3x_read_sensor(float *t, float *h)
{
	uint16_t sd = 1;
	int cnt = 0;

	uint8_t data[6] = {0};

	while ((sd == 1) && cnt <= 3) {
		sd = sht3x_read_data(READ_HIGH_RES, data, 6);
		cnt++;
	}

	if (sd != 0) {
		// bus error or no sensors

		if (t != NULL)
			*t = -1.0;
		if (h != NULL)
			*h = -1.0;

		return -1;
	}

	if (crc8(data, 2) != data[2] || crc8(data+3, 2) != data[5]) {
		INFO("SHT3X: crc error!");

		if (t != NULL)
			*t = -2.0;
		if (h != NULL)
			*h = -2.0;

		return -2;
	}

	if (t != NULL) {
		sd = data[0] << 8 | data[1];
		*t = (-45.0 + 175.0 * (float)sd / 65535.0);
	}

	if (h != NULL) {
		sd = data[3] << 8 | data[4];
		*h = 100.0 * (float)sd / 65535.0;
	}

	return 0;
}

static void swap(float *p, float *q)
{
	float t;

	t = *p;
	*p = *q;
	*q = t;
}

static void sort(float a[], int n)
{
	int i, j;

	for (i = 0; i < n - 1; i++) {

		for (j = 0; j < n - i - 1; j++) {

			if (a[j] > a[j + 1]) {

				swap(&a[j], &a[j + 1]);

			}
		}
	}
}

static float median(float a[], int n)
{
	int m = 0;

	sort(a, n);

	m = (n + 1) / 2 - 1;

	return a[m];
}

/*
 * return float: The temperature in Deg C
 */
struct sht3x_data {
	float temp;
	float humi;
	bool valid;
};

static struct sht3x_data g_data;

float sht3x_get_temp(void)
{
	float t[5], h[5];
	int i;

	if (g_data.valid == 1) {

		g_data.valid = 0;
		return g_data.temp;

	} else {

		for(i = 0; i < 5; i++) {
			sht3x_read_sensor(t+i, h+i);	
			sht3x_delay(1);
		}
		
		g_data.temp = median(t, 5);
		g_data.humi = median(h, 5);
		g_data.valid = 1;

		return g_data.temp;
	}
}

float sht3x_get_humi(void)
{
	float t[5], h[5];
	int i;

	if (g_data.valid == 1) {

		g_data.valid = 0;
		return g_data.humi;

	} else {

		for(i = 0; i < 5; i++) {
			sht3x_read_sensor(t+i, h+i);	
			sht3x_delay(1);
		}
		
		g_data.temp = median(t, 5);
		g_data.humi = median(h, 5);
		g_data.valid = 1;

		return g_data.humi;
	}
	
}

uint8_t sht3x_init(uint8_t scl, uint8_t sda)
{
	wire_begin(scl, sda);

	sht3x_reset();

	return 0;
}
