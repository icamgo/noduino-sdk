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

#include "mhz16.h"

static uint8_t R_CO2_CMD[9] =
	{0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

static uint8_t CALI_ZERO_CMD[9] =
	{0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};

/* 0x07d0 = 2000 */
static uint8_t CALI_SPAN_POINT_CMD[9] =
	{0xFF, 0x01, 0x88, 0x07, 0xD0, 0x00, 0x00, 0x00, 0xA0};

static Stream *serial;
static uint8_t temperature;

void mhz16_begin(Stream * stream)
{
	serial = stream;
}

void mhz16_send_cmd(uint8_t *cmd)
{
	uint8_t i;

	for (i = 0; i < 9; i++) {
		serial->write(cmd[i]);
	}
}

int32_t mhz16_get_co2()
{
	uint8_t i = 0;
	uint8_t buf[12];
	uint32_t start = millis();
	uint8_t av;
	uint32_t ppm = 0;

	mhz16_send_cmd(R_CO2_CMD);

	while (i < 9) {
//			Serial.println(i);
		if (av = serial->available()) {
//			Serial.print("available==");
//			Serial.println(av);
			buf[i] = serial->read();

			if (i == 0 && buf[0] != 0xFF) {
				continue;
			} else {
				i++;
			}
		}

		if (millis() - start > 1000) {
			return -2;
		}
		//delay(10);
	}

//	  for (i=0; i<8; i++) {
//			  Serial.print(buf[i], HEX);
//			  Serial.print(" ");
//	  }
//
//	  Serial.println(buf[8], HEX);

	if (buf[1] == 0x86) {
		if (mhz16_checksum(buf)) {
			ppm = (uint16_t) buf[2] << 8 | buf[3];
			temperature = buf[4] - 40;
			return ppm;
		} else {
			return -3;
		}
	} else {
		return -4;
	}
}

void mhz16_set_zero()
{
	mhz16_send_cmd(CALI_ZERO_CMD);
}

void mhz16_set_span_point()
{
	mhz16_send_cmd(CALI_SPAN_POINT_CMD);
}

uint8_t mhz16_checksum(uint8_t * pkt)
{
	uint8_t i, checksum = 0;

	for (i = 1; i < 8; i++) {
		checksum += pkt[i];
	}

	checksum = 0xff - checksum;
	checksum += 1;
	return checksum;
}

int8_t mhz16_get_temp()
{
	return temperature;
}
