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

float cal_temp(uint32_t Rt)
{
	float temp;
	float LR;
	float HR;
	float T;
	uint8_t i;

	uint8_t Bottom, Top;

	if (Rt < PT100_TABLE[0]) {
		return -274.0;
	}

	if (Rt > PT100_TABLE[99]) {
		return 300.0;
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

uint32_t pt1000_get_rt()
{
	uint32_t rt = 0;
	int a6 = 0, a7 = 0;

	for (int i = 0; i < N_TRY; i++) {

		a6 = adc.read(A6);
		a7 = adc.read(A7);

	#if 0
		Serial.print("ADC6 = ");
		Serial.println(a6);

		Serial.print("ADC7 = ");
		Serial.println(a7);
	#endif
		if (a6 == a7) return 30000;

		// 1.1K x 10
		rt += (uint32_t)(11000.0 / ((float)a6 / a7 - 1.0));

	#if 0
		Serial.print("Rt = ");
		Serial.println(rt);
	#endif

		pt_delay(PT_1MS);
	}

	rt /= N_TRY;

	#if 0
		Serial.print("Rt = ");
		Serial.println(rt);
	#endif

	return rt;
}

/*
 * N_TRY = 1: Time consumption <3ms
 * N_TRY = 5: Time consumption <27ms
 * N_TRY = 18: Time consumption <95ms
 */
float pt1000_get_temp()
{
	uint32_t rt = 0;

	rt = pt1000_get_rt();

#if 0
	Serial.print("Rt = ");
	Serial.println(rt);
#endif

	return cal_temp(rt);
}

void pt1000_init()
{
	adc.set_adcinited(0);
	adc.reference(adcRefVDD);
}
