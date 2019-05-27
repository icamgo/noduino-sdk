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
#include "user_config.h"

float cal_temp(uint32_t Rt)
{
	float temp;
	float LR;
	float HR;
	int T;
	uint8_t i;

	uint8_t Bottom, Top;

	if (Rt < PT100_TABLE[0]) {
		return 0.0;
	}

	if (Rt > PT100_TABLE[210]) {
		return 1.0;
	}

	Bottom = 0;
	Top = 210;

	for (i = 105; (Top - Bottom) != 1;) {
		if (Rt < PT100_TABLE[i]) {
			Top = i;
			i = (Top + Bottom) / 2;
		} else if (Rt > PT100_TABLE[i]) {
			Bottom = i;
			i = (Top + Bottom) / 2;
		} else {
			T = (int)i *5 - 200;
			temp = (float)T;

			return temp;
		}
	}

	T = (int)i *5 - 200;

	LR = PT100_TABLE[Bottom];
	HR = PT100_TABLE[Top];

	temp = (((Rt - LR) * 5) / (HR - LR)) + T;

	return temp;
}

uint32_t get_rt(uint32_t uv)
{
	uint32_t rtd = 0;

	if (uv != 0) {
		rtd = (uint32_t) (10 * 1000.0 * uv / (2489000 - uv));
	} else {
		rtd = 0;
	}
	INFO("rtd = %d\r\n", rtd);
	return rtd;
}

float pt1000_get_temp()
{
	int uv = mcp342x_get_uv();
	int rt = get_rt(uv);

	return cal_temp(rt);
}

irom float cal_temp_x(float R)
{
	float R0 = 1000.0;
	float A = 3.9083E-3;
	float B = -5.775E-7;
	float T;

	R = R / R0;

	//T = (0.0-A + sqrt((A*A) - 4.0 * B * (1.0 - R))) / 2.0 * B; 
	T = 0.0 - A;
	T += sqrt((A * A) - 4.0 * B * (1.0 - R));
	T /= (2.0 * B);

	return T;
}
