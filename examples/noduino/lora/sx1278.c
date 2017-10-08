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

#define FREQ_STEP	61.03515625
/*
 * freq = (uint32_t)((double)434000000 / (double)FREQ_STEP)
 *      = 7094272 = 0x6C8000
 *
 *  lora_freq[0] = msb = (uint8_t) ((freq >> 16) & 0xff)
 *  lora_freq[1] = mid = (uint8_t) ((freq >> 8) & 0xff)
 *  lora_freq[2] = lsb = (uint8_t) (freq & 0xff)
 *
 */
uint8_t lora_freq[3] = { 0x6c, 0x80, 0x00 };	// set to 434MHz

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


///////////////////////////////////////////////////////////////

void sx1278_set_rf_freq(void)
{
	sx1278_write_reg(REG_LR_FRFMSB, lora_freq[0]);
	sx1278_write_reg(REG_LR_FRFMID, lora_freq[1]);
	sx1278_write_reg(REG_LR_FRFLSB, lora_freq[2]);
}

void sx1278_set_nb_trig_peaks(uint8_t value)
{
	uint8_t d;
	d = sx1278_read_reg(0x31);
	d = (d & 0xF8) | value;
	sx1278_write_reg(0x31, d);
}

void sx1278_set_spread_fact(uint8_t factor)
{
	uint8_t d;
	sx1278_set_nb_trig_peaks(3);
	d = sx1278_read_reg(REG_LR_MODEMCONFIG2);
	d = (d & RFLR_MODEMCONFIG2_SF_MASK) | (factor << 4);
	sx1278_write_reg(REG_LR_MODEMCONFIG2, d);
}

void sx1278_set_error_coding(uint8_t value)
{
	uint8_t d;
	d = sx1278_read_reg(REG_LR_MODEMCONFIG1);
	d = (d & RFLR_MODEMCONFIG1_CODINGRATE_MASK)
	    | (value << 1);
	sx1278_write_reg(REG_LR_MODEMCONFIG1, d);
// LoRaSettings.error_coding = value;
}

void sx1278_set_bandwidth(uint8_t bw)
{
	uint8_t d;
	d = sx1278_read_reg(REG_LR_MODEMCONFIG1);
	d = (d & RFLR_MODEMCONFIG1_BW_MASK) | (bw << 4);
	sx1278_write_reg(REG_LR_MODEMCONFIG1, d);
// LoRaSettings.SignalBw = bw;
}

void sx1278_set_head_on(bool enable)
{
	uint8_t rxd;
	rxd = sx1278_read_reg(REG_LR_MODEMCONFIG1);
	rxd = (rxd & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK)
				| (enable);
	sx1278_write_reg(REG_LR_MODEMCONFIG1, rxd);
}

void sx1278_set_payload_len(uint8_t value)
{
	sx1278_write_reg(REG_LR_PAYLOADLENGTH, value);
}

void sx1278_set_symb_timeout(unsigned int value)
{
	uint8_t d[2];
	d[0] = sx1278_read_reg(REG_LR_MODEMCONFIG2);
	d[1] = sx1278_read_reg(REG_LR_SYMBTIMEOUTLSB);
	d[0] = (d[0] & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK)
	    | ((value >> 8) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK);
	d[1] = value & 0xFF;
	sx1278_write_reg(REG_LR_MODEMCONFIG2, d[0]);
	sx1278_write_reg(REG_LR_SYMBTIMEOUTLSB, d[1]);
}

void sx1278_set_mobi_node(bool enable)
{
	uint8_t d;
	d = sx1278_read_reg(REG_LR_MODEMCONFIG3);
	d = (d & RFLR_MODEMCONFIG3_MOBILE_NODE_MASK)
	    | (enable << 3);
	sx1278_write_reg(REG_LR_MODEMCONFIG3, d);
}

void lora_rx_mode(void)
{
	sx1278_set_opmode(STANDBY);
	sx1278_write_reg(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);
	sx1278_write_reg(REG_LR_HOPPERIOD, PACKET_MIAX_Value);
	sx1278_write_reg(REG_LR_DIOMAPPING1, 0X00);
	sx1278_write_reg(REG_LR_DIOMAPPING2, 0X00);
	sx1278_set_opmode(RX);
}

void sx1278_set_crc_on(bool enable)
{
	uint8_t d;
	d = sx1278_read_reg(REG_LR_MODEMCONFIG2);
	d = (d & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK)
	    | (enable << 2);
	sx1278_write_reg(REG_LR_MODEMCONFIG2, d);
}

void sx1278_set_rfmode(rfmode_t opm)
{
	uint8_t op_mod;

	sx1278_set_opmode(SLEEP);

	op_mod = sx1278_read_reg(REG_LR_OPMODE);
	op_mod &= 0x7F;
	op_mod |= (uint8_t)opm;
	sx1278_write_reg(REG_LR_OPMODE, op_mod);

	sx1278_set_opmode(STANDBY);
}

uint8_t sx1278_get_rfmode()
{
	uint8_t op_mod;
	op_mod = sx1278_read_reg(REG_LR_OPMODE);
	op_mod &= 0x80;
	return op_mod;
}

uint8_t sx1278_get_opmode()
{
	uint8_t opm = sx1278_read_reg(REG_LR_OPMODE);
	opm &= 0x7;
	return opm;
}

void sx1278_set_opmode(opmode_t opmode)
{
	uint8_t opm;
	opm = sx1278_read_reg(REG_LR_OPMODE);
	opm &= 0xf8;
	opm |= (uint8_t)opmode;
	sx1278_write_reg(REG_LR_OPMODE, opm);
}

void sx1278_set_rf_power(uint8_t power)
{
	sx1278_write_reg(REG_LR_PADAC, 0x87);
	sx1278_write_reg(REG_LR_PACONFIG, power_data[power]);
}

void sx1278_init(void)
{
	sx1278_set_rfmode(LORA);

	sx1278_write_reg(REG_LR_DIOMAPPING1, GPIO_VARE_1);
	sx1278_write_reg(REG_LR_DIOMAPPING2, GPIO_VARE_2);

	sx1278_set_rf_freq();
	sx1278_set_rf_power(rf_power);

	sx1278_set_spread_fact(spread_fact);
	sx1278_set_error_coding(code_rate);
	sx1278_set_crc_on(true);
	sx1278_set_bandwidth(lora_bw);
	sx1278_set_head_on(false);
	sx1278_set_payload_len(0xff);		// 0x22 timeout interrupt
	sx1278_set_symb_timeout(0x3FF);
	sx1278_set_mobi_node(true);

	lora_rx_mode();
}

void sx1278_send_data(uint8_t *data, uint8_t len)
{
	sx1278_set_opmode(STANDBY);
	sx1278_write_reg(REG_LR_HOPPERIOD, 0);
	sx1278_write_reg(REG_LR_IRQFLAGSMASK, IRQN_TXD_Value);	//enable tx interrupt
	sx1278_write_reg(REG_LR_PAYLOADLENGTH, len);
	sx1278_write_reg(REG_LR_FIFOTXBASEADDR, 0);
	sx1278_write_reg(REG_LR_FIFOADDRPTR, 0);

	spi_write_buf(0x80, data, len);

	sx1278_write_reg(REG_LR_DIOMAPPING1, 0x40);
	sx1278_write_reg(REG_LR_DIOMAPPING2, 0x00);
	sx1278_set_opmode(TX);
}
