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

#include "em_aes.h"

/* 
 * Encrypt a plaintext message (32 bytes) using the AES CBC block cipher
 * mode with a 128 bits key and initial vector (iv) of 16 bytes
 *
*/
const uint8_t msg[32] __attribute__((aligned(4))) = {0x47, 0x4F, 0x33, 0x00, 0x00, 0x00, 0x02, 0xC5,
                            0x6E, 0x3B, 0xBA, 0x00, 0x8F, 0x0E, 0x29, 0x02,
                            0xE5, 0x75, 0x05, 0x15, 0x00, 0x41, 0x4D, 0x8C,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const uint8_t key[16] __attribute__((aligned(4))) = {0x64, 0x63, 0x47, 0x4F, 0x64, 0x63, 0x47, 0x4F,
                      	 0x64, 0x63, 0x47, 0x4F, 0x64, 0x63, 0x47, 0x4F};

const uint8_t iv[16] __attribute__((aligned(4))) = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                     	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

uint8_t e_msg[32] __attribute__((aligned(4))); /* Output buffer for encrypted data (ciphertext). */

uint8_t d_key[16] __attribute__((aligned(4)));

void setup()
{
	Serial.setRouteLoc(1);
	Serial.begin(115200);

	CMU_ClockEnable(cmuClock_AES, true);
} 

void print_msg(uint8_t *pkt)
{
	for (int i = 0; i < 32; i++) {

		if (pkt[i] < 16)
			Serial.print("0");

		Serial.print(pkt[i], HEX);

		Serial.print(" ");
	}

	Serial.println("");
}

void loop()
{
	int start = 0, end = 0;

	Serial.println("Testing the AES Engine... ");

	start = millis();

	AES_CBC128(e_msg, msg, 32, key, iv, true);  /* true means encrypt. */

	end = millis();

	Serial.print("Encrypted 32 bytes spend: ");
	Serial.print(end - start);
	Serial.println("ms");

	Serial.println("The Encrypyted MSG:");
	print_msg(e_msg);

	AES_DecryptKey128(d_key, key);

	AES_CBC128(e_msg, e_msg, 32, d_key, iv, false);
	Serial.println("The Decrypyted MSG:");
	print_msg(e_msg);

	delay(6000);
}
