#include "em_aes.h"

#define BLK_B0_LEN                   16
static uint8_t blk_b0[] __attribute__ ((aligned(4))) = {
	0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t iv[] __attribute__ ((aligned(4))) = {
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t aes_out[44] __attribute__ ((aligned(4)));

/*
 * [IN]  p          Data p
 * [IN]  plen       Data p plen
 * [IN]  key        key to be used
 * [OUT] mic        compute_d MIC field
 */
void compute_mic(const uint8_t *p, uint16_t plen, const uint8_t *key,
					uint32_t *mic)
{
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

	AES_CBC128(aes_out, p+12, 16, key, blk_b0, true);  /* true means encrypt. */

	*mic = (uint32_t) ((uint32_t) aes_out[3] << 24 | (uint32_t) aes_out[2] << 16 |
			(uint32_t) aes_out[1] << 8 | (uint32_t) aes_out[0]);
}

void payload_encrypt(const uint8_t *p, uint16_t plen, const uint8_t *key,
						uint8_t *aes_out)
{
	memset(iv, 16, 0);

	iv[5] = p[6];
	iv[6] = p[7]; 
	iv[7] = p[8];
	iv[8] = p[9];
	iv[9] = p[10];

	iv[10] = p[plen-8]; 
	iv[11] = p[plen-7];
	iv[12] = p[plen-6]; 
	iv[13] = p[plen-5]; 

	iv[15] = plen & 0xFF;

	AES_CBC128(aes_out, p, plen, key, iv, true);  /* true means encrypt. */
}

void payload_decrypt(const uint8_t *p, uint16_t plen, const uint8_t *key,
						uint8_t *aes_out)
{

}
