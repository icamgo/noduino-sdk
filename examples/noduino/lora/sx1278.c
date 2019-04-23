/*
 *  Copyright (c) 2017 - 2025 MaiKe Labs
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

#include "sx1278.h"


uint8_t rf_power = 7;
uint8_t power_data[8] = {
	0X80, 0X80, 0X80, 0X83, 0X86, 0x89, 0x8c, 0x8f
};


/*
 * Lora Spreading Factor:
 *   6: 64, 7: 128, 8: 256, 9: 512, 10: 1024,
 *   11: 2048, 12: 4096  chips
 *
*/
uint8_t spread_fact = 12;


 /*
  * Lora Error Coding:
  *
  *   1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8
 */
uint8_t code_rate = 2;

/*
 * Lora band width 
 *   0: 7.8 kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz,
 *   4: 31.2 kHz,5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz,
 *   8: 250 kHz, 9: 500 kHz, other: Reserved
*/
uint8_t lora_bw = 7;							

uint8_t sx1278_read_reg(uint8_t addr)
{
	return spi_read_reg(addr & 0x7f);
}

void sx1278_write_reg(uint8_t addr, uint8_t buffer)
{
	spi_write_reg(addr | 0x80, buffer);
}

void sx1278_write_buf(uint8_t addr, uint8_t *data, uint8_t len)
{
	spi_write_buf(addr | 0x80, data, len);
}

void sx1278_set_lora_detect_opti(uint8_t value)
{
	uint8_t d;
	d = sx1278_read_reg(REG_DETECTIONOPTIMIZE);
	d = (d & 0xF8) | value;
	sx1278_write_reg(REG_DETECTIONOPTIMIZE, d);
}

uint8_t sx1278_get_spread_fact()
{
	uint8_t d;
	d = sx1278_read_reg(REG_MODEMCONFIG2);
	return d >> 4;
}

uint16_t sx1278_get_preamble_len()
{
	uint8_t msb = sx1278_read_reg(REG_PREAMBLEMSB);
	uint8_t lsb = sx1278_read_reg(REG_PREAMBLELSB);
	uint16_t l = 0;

	l = (msb << 8) | lsb;
	return l;
}

int sx1278_get_rssi()
{
	/* LF = -164, HF = -157  */
	int off = -164;
	return off + sx1278_read_reg(REG_RSSIVALUE);
}

void sx1278_set_spread_fact(uint8_t factor)
{
	uint8_t d;
	d = sx1278_read_reg(REG_MODEMCONFIG2);
	d = (d & MODEMCONFIG2_SF_MASK) | (factor << 4);
	sx1278_write_reg(REG_MODEMCONFIG2, d);

	if (factor == 6) {
		sx1278_write_reg(REG_DETECTIONOPTIMIZE, 0x5);
		sx1278_write_reg(REG_DETECTIONTHRESHOLD, 0xC);
	} else {
		sx1278_write_reg(REG_DETECTIONOPTIMIZE, 0x3);
		sx1278_write_reg(REG_DETECTIONTHRESHOLD, 0xA);
	}
}

void sx1278_set_coding_rate(uint8_t value)
{
	uint8_t d;
	d = sx1278_read_reg(REG_MODEMCONFIG1);
	d = (d & MODEMCONFIG1_CODINGRATE_MASK)
	    | (value << 1);
	sx1278_write_reg(REG_MODEMCONFIG1, d);
}

void sx1278_set_bandwidth(uint8_t bw)
{
	uint8_t d;
	d = sx1278_read_reg(REG_MODEMCONFIG1);
	d = (d & MODEMCONFIG1_BW_MASK) | (bw << 4);
	sx1278_write_reg(REG_MODEMCONFIG1, d);
}

void sx1278_set_head_off(bool enable)
{
	uint8_t rxd;
	rxd = sx1278_read_reg(REG_MODEMCONFIG1);
	rxd = (rxd & MODEMCONFIG1_IMPLICITHEADER_MASK) | (enable);
	sx1278_write_reg(REG_MODEMCONFIG1, rxd);
}

void sx1278_set_payload_len(uint8_t value)
{
	sx1278_write_reg(REG_PAYLOADLENGTH, value);
}

void sx1278_set_symb_timeout(unsigned int value)
{
	uint8_t d[2];
	d[0] = sx1278_read_reg(REG_MODEMCONFIG2);
	d[1] = sx1278_read_reg(REG_SYMBTIMEOUTLSB);
	d[0] = (d[0] & MODEMCONFIG2_SYMBTIMEOUTMSB_MASK)
	    | ((value >> 8) & ~MODEMCONFIG2_SYMBTIMEOUTMSB_MASK);
	d[1] = value & 0xFF;
	sx1278_write_reg(REG_MODEMCONFIG2, d[0]);
	sx1278_write_reg(REG_SYMBTIMEOUTLSB, d[1]);
}

void sx1278_set_lowdatarate_opti(bool enable)
{
	uint8_t d;
	d = sx1278_read_reg(REG_MODEMCONFIG3);
	d = (d & MODEMCONFIG3_LOWDATARATE_OPTI_MASK)
	    | (enable << 3);
	sx1278_write_reg(REG_MODEMCONFIG3, d);
}

void lora_rx_mode(void)
{
	sx1278_set_opmode(STANDBY);
	sx1278_write_reg(REG_IRQFLAGSMASK, IRQN_RXD_Value);
	sx1278_write_reg(REG_HOPPERIOD, PACKET_MIAX_Value);
	//sx1278_write_reg(REG_DIOMAPPING1, 0X00);
	//sx1278_write_reg(REG_DIOMAPPING2, 0X00);
	sx1278_set_opmode(RX);
}

void sx1278_set_crc_on(bool enable)
{
	uint8_t d;
	d = sx1278_read_reg(REG_MODEMCONFIG2);
	d = (d & MODEMCONFIG2_RXPAYLOADCRC_MASK)
	    | (enable << 2);
	sx1278_write_reg(REG_MODEMCONFIG2, d);
}

void sx1278_set_rfmode(rfmode_t opm)
{
	uint8_t op_mod;

	sx1278_set_opmode(SLEEP);

	op_mod = sx1278_read_reg(REG_OPMODE);
	op_mod &= 0x7F;
	op_mod |= (uint8_t)opm;
	sx1278_write_reg(REG_OPMODE, op_mod);

	sx1278_set_opmode(STANDBY);
}

uint8_t sx1278_get_rfmode()
{
	uint8_t op_mod;
	op_mod = sx1278_read_reg(REG_OPMODE);
	op_mod &= 0x80;
	return op_mod;
}

uint8_t sx1278_get_opmode()
{
	uint8_t opm = sx1278_read_reg(REG_OPMODE);
	opm &= 0x7;
	return opm;
}

void sx1278_set_syncword(uint8_t sw)
{
	sx1278_write_reg(REG_SYNCWORD, sw);
}

uint8_t sx1278_get_syncword()
{
	return sx1278_read_reg(REG_SYNCWORD);
}

void sx1278_get_rf_freq(uint8_t *d)
{
	d[0] = sx1278_read_reg(REG_FRFMSB);
	d[1] = sx1278_read_reg(REG_FRFMID);
	d[2] = sx1278_read_reg(REG_FRFLSB);
}

void sx1278_set_opmode(opmode_t opmode)
{
	uint8_t opm;
	opm = sx1278_read_reg(REG_OPMODE);
	opm &= 0xf8;
	opm |= (uint8_t)opmode;
	sx1278_write_reg(REG_OPMODE, opm);
}

void sx1278_set_rf_power(uint8_t power)
{
	//sx1278_write_reg(REG_PADAC, 0x87);
	sx1278_write_reg(REG_PACONFIG, 0x8F);
}

/*
 * 0x12 -- 150mA
 * 0x10 -- 130mA
 * 0x0B -- 100mA
 * 0x1B -- 240mA
 */
void sx1278_set_max_current(uint8_t rate)
{
	rate |= 0b00100000;
	sx1278_write_reg(REG_OCP, rate);
}

/*
 * 0x0 - 3.4ms
 * 0x1 - 2ms
 * 0x2 - 1ms
 * 0x3 - 500us
 * 0x4 - 250us
 * 0x5 - 125us
 * 0x6 - 100us
 * 0x7 - 62us
 * 0x8 - 50us
 * 0x9 - 40us
 * 0xa - 31us
 * 0xb - 25us
 * 0xc - 20us
 * 0xd - 15us
 * 0xe - 12us
 * 0xf - 10us
 */
void sx1278_set_pa_ramp()
{
	sx1278_write_reg(REG_PARAMP, 0x08);
	/* set 50us PA ramp-up time */
}

///////////////////////////////////////////////////////////////
#define FREQ_STEP	61.03515625
/*
 * freq = (uint32_t)((double)434000000 / (double)FREQ_STEP)
 *      = 7094272 = 0x6C8000 			 // 434.0MHz
 *
 *  const uint32_t CH_00_433 = 0x6C5333; // 433.3MHz
 *  const uint32_t CH_01_433 = 0x6C6666; // 433.6MHz
 *  const uint32_t CH_02_433 = 0x6C7999; // 433.9MHz
 *  const uint32_t CH_03_433 = 0x6C9333; // 434.3MHz
 *
 *  const uint32_t CH_00_470 = 0x758000; // 470.0MHz
 *
 *  lora_freq[0] = msb = (uint8_t) ((freq >> 16) & 0xff)
 *  lora_freq[1] = mid = (uint8_t) ((freq >> 8) & 0xff)
 *  lora_freq[2] = lsb = (uint8_t) (freq & 0xff)
 *
 */
uint8_t lora_freq[3] = { 0x6C, 0x93, 0x33 };	// set to 434.3MHz

void sx1278_set_rf_freq(void)
{
	sx1278_write_reg(REG_FRFMSB, lora_freq[0]);
	sx1278_write_reg(REG_FRFMID, lora_freq[1]);
	sx1278_write_reg(REG_FRFLSB, lora_freq[2]);
}

void sx1278_init(void)
{
	sx1278_set_opmode(STANDBY);
	sx1278_set_rfmode(LORA);

	//sx1278_write_reg(REG_DIOMAPPING1, GPIO_VARE_1);
	//sx1278_write_reg(REG_DIOMAPPING2, GPIO_VARE_2);

	sx1278_set_pa_ramp();		// 50us
	sx1278_set_rf_freq();		// 434.3MHz

#if 0
	sx1278_set_bandwidth(7);	// 125KHz
	sx1278_set_coding_rate(1);
	sx1278_set_head_off(false);
#else
	// 125KHz, 4/5, Explicit Header
	sx1278_write_reg(REG_MODEMCONFIG1, 0x72);
#endif

#if 0
	sx1278_set_spread_fact(12);
	sx1278_set_crc_on(true);
#else
	// SF=12, TxContin single pkt, crc on, RX timeout msb - 0x0
	sx1278_write_reg(REG_MODEMCONFIG2, 0xC4);
#endif

	//sx1278_set_lowdatarate_opti(true);

	/*
	 * LowDataRateOptimize = 1, mandated for when symbol len > 16ms
	 * AgcAutoOn = 1, LNA gain set by the internal AGC loop
	*/
	sx1278_write_reg(REG_MODEMCONFIG3, 0x0C);

	sx1278_set_syncword(0x34);

	sx1278_set_max_current(0x1B);	// 240mA
	sx1278_set_rf_power(rf_power);

	//sx1278_set_symb_timeout(0x3FF);
	//lora_rx_mode();
}

void sx1278_send_data(uint8_t *data, uint8_t len)
{
	sx1278_set_opmode(STANDBY);
	sx1278_write_reg(REG_HOPPERIOD, 0);
	sx1278_write_reg(REG_IRQFLAGSMASK, IRQN_TXD_Value);	//enable tx interrupt

	sx1278_write_reg(REG_PAYLOADLENGTH, len);

	sx1278_write_reg(REG_FIFOTXBASEADDR, 0x80);

	sx1278_write_reg(REG_FIFOADDRPTR, 0x80);
	sx1278_write_buf(REG_FIFO, data, len);

//	sx1278_write_reg(REG_DIOMAPPING1, 0x40);
//	sx1278_write_reg(REG_DIOMAPPING2, 0x00);
	sx1278_set_opmode(TX);
}
