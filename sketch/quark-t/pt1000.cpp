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
#include "pt1000.h"

#define REF_V		2491000.0

float cal_temp(uint32_t Rt)
{
	float temp;
	float LR;
	float HR;
	float T;
	uint8_t i;

	uint8_t Bottom, Top;

	if (Rt < PT100_TABLE[0]) {
		return 0.0;
	}

	if (Rt > PT100_TABLE[99]) {
		return 1.0;
	}

	Bottom = 0;
	Top = 99;

	for (i = 49; (Top - Bottom) != 1;) {
		if (Rt < PT100_TABLE[i]) {
			Top = i;
			i = (Top + Bottom) / 2;
		} else if (Rt > PT100_TABLE[i]) {
			Bottom = i;
			i = (Top + Bottom) / 2;
		} else {
			T = i * 5.0 - 200.0;

			return T;
		}
	}

	T = i * 5.0 - 200.0;

	LR = PT100_TABLE[Bottom];
	HR = PT100_TABLE[Top];

	temp = (((Rt - LR) * 5.0) / (HR - LR)) + T;

	return temp;
}

uint32_t pt1000_get_rt(float uv)
{
	uint32_t rtd = 0;

	if (uv != 0) {
		rtd = 10.0 * 1000.0 * uv / (REF_V - uv);
	} else {
		rtd = 0;
	}
	Serial.print("Rtd = ");
	Serial.println(rtd);
	return rtd;
}

float pt1000_get_temp()
{
	float uv = get_adc_uv();
	uint32_t rt = pt1000_get_rt(uv);

	return cal_temp(rt);
}

void pt1000_init()
{
	adc_init();
}
