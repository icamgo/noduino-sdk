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

	float myFloat = 13.245;

	uint16_t myfp16 = 0;

	float2halfp(&myfp16, &myFloat, 1);		// convert 1 float to fp16

	Serial.println("Testing the general function of float16... ");

	Serial.print("Convert fp32=13.245 to fp16, HEX: ");
	Serial.println(myfp16, HEX);
	 
	float myfp32 = 0;
	halfp2float(&myfp32, &myfp16, 1);		// recover from 1 fp16 to float

	Serial.print("fp16 to float: ");
	Serial.println((double)myfp32, 3);

	Serial.println("---------------------------------------------------");
}

void hex2fp16(uint16_t h)
{
	char omsg[32];

	float x = 0;
	halfp2float(&x, &h, 1);

	Serial.print("0x");
	Serial.print(h, HEX);
	Serial.print(" = ");
	Serial.println((double)x, 10);
}


void float2fp16hex(float f32)
{
	uint16_t f = 0;

	float2halfp(&f, &f32, 1);

	Serial.print((double)f32, 10);
	Serial.print(" = 0x");
	Serial.println((uint16_t)f, HEX);
}

void setup()
{
	Serial.setRouteLoc(1);
	Serial.begin(115200);
} 


void loop()
{
	__fp16 f1 = 0.232;
	
	Serial.println("---------------------------------------------------");
	Serial.println("Testing the float16, supported by gcc... ");

	Serial.print("sizeof(fp16): ");
	Serial.println(sizeof(f1));

	Serial.print("f1 = ");
	Serial.println((double)f1, 4);
	Serial.println("---------------------------------------------------");

	// test the function
	test_fp16();

	Serial.println("----- Test the [0, 1] -----");
	// 0x0 = 0
	// 0x3c00 = 1.0
	float2fp16hex(0);

	hex2fp16(0x2);
	hex2fp16(0x3);
	hex2fp16(0x4);

	hex2fp16(0x0080);
	hex2fp16(0x0800);

	hex2fp16(0x2000);
	hex2fp16(0x2001);
	hex2fp16(0x2002);
	hex2fp16(0x3000);

	hex2fp16(0x3bfd);
	hex2fp16(0x3bfe);
	hex2fp16(0x3bff);

	float2fp16hex(1);

	Serial.println("----- Test the [-1, 0] -----");
	hex2fp16(0x8002);
	hex2fp16(0x8003);
	hex2fp16(0x8004);

	hex2fp16(0x8080);
	hex2fp16(0x8800);

	hex2fp16(0xA000);
	hex2fp16(0xA001);
	hex2fp16(0xA002);

	hex2fp16(0xB000);

	hex2fp16(0xbbfd);
	hex2fp16(0xbbfe);
	hex2fp16(0xbbff);


	Serial.println("----- Test the [1, 2048] -----");
	// 0x3c00 = 1, 0x6800 = 2048
	hex2fp16(0x3c01);
	hex2fp16(0x3c02);

	hex2fp16(0x3CFD);		// 1.247
	hex2fp16(0x3CFE);		// 1.248
	hex2fp16(0x3CFF);		// 1.249

	float2fp16hex(3.676);
	float2fp16hex(3.677);
	float2fp16hex(3.678);

	float2fp16hex(3.544);
	float2fp16hex(3.545);
	float2fp16hex(3.546);

	hex2fp16(0x3d00);
	hex2fp16(0x3e00);

	hex2fp16(0x4000);
	hex2fp16(0x4001);
	hex2fp16(0x4002);

	hex2fp16(0x5000);
	hex2fp16(0x5001);
	hex2fp16(0x5002);
	float2fp16hex(32.1234);
	float2fp16hex(32.2234);
	float2fp16hex(32.2254);

	hex2fp16(0x6000);
	hex2fp16(0x6001);
	hex2fp16(0x6002);

	hex2fp16(0x67fe);
	hex2fp16(0x67ff);

	float2fp16hex(2048);

	Serial.println("----- Test the [2048, NaN] -----");
	// 0x6800 = 2048, 0x6c00 = 4096
	float2fp16hex(4096);

	float2fp16hex(8192);

	float2fp16hex(16384);

	float2fp16hex(32768);

	float2fp16hex(65504);

	hex2fp16(0x7bfd);		// 65440
	hex2fp16(0x7bfe);		// 65472
	hex2fp16(0x7bff);		// 65504
	hex2fp16(0x7c00);
	hex2fp16(0x7c01);
	hex2fp16(0x7d00);
	hex2fp16(0x7e00);
	hex2fp16(0x7f00);
	hex2fp16(0x8000);

	delay(5000);
}
