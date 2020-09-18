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
uint8_t msg[32] __attribute__((aligned(4))) = {
	0x42, 0x43, 0x44, 0x01, 0x02, 0x03, 0x04, 0x05,
	0x6E, 0x3B, 0xBA, 0x01, 0x8F, 0x0E, 0x29, 0x02,
	0xE5, 0x75, 0x05, 0x15, 0x00, 0x41, 0x4D, 0x8C,
	0x00, 0x41, 0x4D, 0x8C, 0x02, 0x03, 0x02, 0x03,
};

const uint8_t key[16] __attribute__((aligned(4))) = {
	0x64, 0x63, 0x47, 0x4F, 0x64, 0x63, 0x47, 0x4F,
	0x64, 0x63, 0x47, 0x4F, 0x64, 0x63, 0x47, 0x4F
};

const uint8_t iv[16] __attribute__((aligned(4))) = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};

void print_msg(uint8_t *pkt, int len);

#define BLK_B0_LEN                   16
static uint8_t blk_b0[] __attribute__ ((aligned(4))) = {
	0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t crypto_out[32] __attribute__ ((aligned(4)));

/*
 * [IN]  p          Data p
 * [IN]  plen       Data p plen
 * [IN]  key        AES key to be used
 * [OUT] mic        compute_d MIC field
 */
void compute_mic(const uint8_t *p, uint16_t plen, const uint8_t *key,
					uint32_t *mic)
{
	uint8_t *out = crypto_out;

	blk_b0[5] = p[6];
	blk_b0[6] = p[7];
	blk_b0[7] = p[8];
	blk_b0[8] = p[9];
	blk_b0[9] = p[10];

	blk_b0[10] = p[plen-8];
	blk_b0[11] = p[plen-7];
	blk_b0[12] = p[plen-6];
	blk_b0[13] = p[plen-5];

	blk_b0[15] = plen & 0xFF;

	AES_CBC128(out, p+4, 16, key, blk_b0, true);  /* true means encrypt. */

	print_msg(out, 32);

	*mic = (uint32_t) ((uint32_t) out[3] << 24 | (uint32_t) out[2] << 16 |
			(uint32_t) out[1] << 8 | (uint32_t) out[0]);
}

uint8_t e_msg[44] __attribute__((aligned(4))); /* Output buffer for encrypted data (ciphertext). */
uint8_t d_key[16] __attribute__((aligned(4)));

void setup()
{
	Serial.setRouteLoc(1);
	Serial.begin(115200);

	CMU_ClockEnable(cmuClock_AES, true);
}

void print_msg(uint8_t *pkt, int len)
{
	for (int i = 0; i < len; i++) {

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

	int len = 33;

	//////////////////////////////////////////
	uint32_t mic = 0;
	Serial.println("Testing the MIC:");
	compute_mic(msg, 32, key, &mic);

	print_msg((uint8_t *)&mic, 4);
	Serial.println("");
	//////////////////////////////////////////


	Serial.println("Testing the AES Engine... ");

	start = millis();

	/*
	 * len < 16: do not encrypt any data
	 * len = [16, 32): only encrypt the first 16bytes
	 * len = [32, 64): only encrypt the 32bytes
	 *
	 * true means encrypt.
	*/
	AES_CBC128(e_msg, msg, len, key, iv, true);

	end = millis();

	Serial.print("Encrypted 28 bytes spend: ");
	Serial.print(end - start);
	Serial.println("ms");

	Serial.println("The Encrypyted MSG:");
	print_msg(e_msg, len);

	AES_DecryptKey128(d_key, key);
	AES_CBC128(e_msg, e_msg, len, d_key, iv, false);

	Serial.println("The Decrypyted MSG:");
	print_msg(e_msg, len);

	delay(6000);
}
