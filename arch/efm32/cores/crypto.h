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

uint8_t ae33kk[] __attribute__ ((aligned(4))) = {
	0x01, 0x02, 0x02, 0x03, 0x05, 0x07, 0x08, 0x08,
	0x02, 0x01, 0x02, 0x03, 0x05, 0x07, 0x08, 0x09
};

uint8_t d_key[16] __attribute__((aligned(4)));

uint8_t pld_outin[16] __attribute__((aligned(4)));

////////////////////////////////////////////////////////////////////////////
void crypto_init();
bool set_pkt_mic(uint8_t *p, int len);
bool check_pkt_mic(const uint8_t *p, int len);
uint8_t *payload_encrypt(const uint8_t *p, uint16_t plen, const uint8_t *kk);
uint8_t *payload_decrypt(const uint8_t *p, uint16_t plen, const uint8_t *kk);
////////////////////////////////////////////////////////////////////////////

void compute_mic(const uint8_t *p, uint16_t plen, const uint8_t *kk,
					uint32_t *mic);

void crypto_init()
{
	CMU_ClockEnable(cmuClock_AES, true);
}

bool set_pkt_mic(uint8_t *p, int len)
{
	uint32_t mic = 0;
	uint8_t *p_mic = (uint8_t *)&mic;

	if (NULL == p) return false;

	if (p[2] == 0x33 && len == 36) {

		compute_mic(p, len, ae33kk, &mic);

		p[len-4] = p_mic[0]; p[len-3] = p_mic[1];
		p[len-2] = p_mic[2]; p[len-1] = p_mic[3];

		return true;

	} else {

		return false;
	}
}

bool check_pkt_mic(const uint8_t *p, int len)
{
	uint32_t mic = 0;
	uint8_t *p_mic = (uint8_t *)&mic;

	if (NULL == p) return false;

	if (p[2] == 0x33 && len == 36) {

		compute_mic(p, len, ae33kk, &mic);

		if (p_mic[0] == p[len-4] && p_mic[1] == p[len-3] &&
			p_mic[2] == p[len-2] && p_mic[3] == p[len-1]) {

			return true;
		} else {
			return false;
		}

	} else {

		return true;
	}
}

uint8_t *payload_encrypt(const uint8_t *p, uint16_t plen, const uint8_t *kk)
{
	memset(iv, 0, 16);
	iv[0] = 0x55;

	memcpy(iv+5, p, 5);

	iv[10] = 0x6;
	iv[11] = 0x4;
	iv[12] = 0x2;
	iv[13] = 0x2;

	iv[15] = plen & 0xFF;

	memcpy(kk, p, 7);

	memset(pld_outin, 0, 16);
	memcpy(pld_outin, p+7, 8);
	memcpy(pld_outin+8, p+16, 8);

	AES_CBC128(pld_outin, pld_outin, 16, kk, iv, true);

	memcpy(p+7, pld_outin, 8);
	memcpy(p+16, pld_outin+8, 8);

	return (uint8_t *)pld_outin;
}

uint8_t *payload_decrypt(const uint8_t *p, uint16_t plen, const uint8_t *kk)
{
	memset(iv, 0, 16);
	iv[0] = 0x55;

	memcpy(iv+5, p, 5);

	iv[10] = 0x6;
	iv[11] = 0x4;
	iv[12] = 0x2;
	iv[13] = 0x2;

	iv[15] = plen & 0xFF;

	memcpy(kk, p, 7);

	memset(pld_outin, 0, 16);
	memcpy(pld_outin, p+7, 8);
	memcpy(pld_outin+8, p+16, 8);

	AES_DecryptKey128(d_key, kk);
	AES_CBC128(pld_outin, pld_outin, 16, d_key, iv, false);

	memcpy(p+7, pld_outin, 8);
	memcpy(p+16, pld_outin+8, 8);

	return (uint8_t *)pld_outin;
}

/*
 * [IN]  p          Data p
 * [IN]  plen       Data p plen
 * [IN]  kk         key to be used
 * [OUT] mic        compute_d MIC field
 */
void compute_mic(const uint8_t *p, uint16_t plen, const uint8_t *kk,
					uint32_t *mic)
{
	// NOTE: plen >=32 !!

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

	memcpy(kk, p, 7);

	AES_CBC128(aes_out, p+12, 16, kk, blk_b0, true);  /* true means encrypt. */

	*mic = (uint32_t) ((uint32_t) aes_out[3] << 24 | (uint32_t) aes_out[2] << 16 |
			(uint32_t) aes_out[1] << 8 | (uint32_t) aes_out[0]);
}

