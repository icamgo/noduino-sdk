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

#include "Arduino.h"

#include  "ieeehalfprecision.c"

void test_fp16()
{

	float myFloat = 1.245;

	uint16_t myfp16 = 0;

	float2halfp(&myfp16, &myFloat, 1);		// convert 1 float to fp16

	Serial.println("Testing the float16 function... ");

	Serial.print("Convert fp32=1.245 to fp16, HEX: ");
	Serial.println(myfp16, HEX);
	 
	float myfp32 = 0;
	halfp2float(&myfp32, &myfp16, 1);		// recover from 1 fp16 to float

	Serial.print("fp16 to float: ");
	Serial.println((double)myfp32, 3);
}

void setup()
{
	Serial.setRouteLoc(1);
	Serial.begin(115200);
} 


void loop()
{
	__fp16 f1 = 0.232;
	
	Serial.println("Testing the float16, supported by gcc... ");

	Serial.print("sizeof(fp16): ");
	Serial.println(sizeof(f1));

	Serial.print("f1 = ");
	Serial.println((double)f1, 4);

	// test the function
	test_fp16();

	delay(5000);
}
