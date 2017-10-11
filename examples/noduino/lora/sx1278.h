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

#ifndef __SX1278_H__
#define __SX1278_H__

#include "noduino.h"

#define REG_FIFO                                  0x00
// Common settings
#define REG_OPMODE                                0x01
#define REG_BANDSETTING                           0x04
#define REG_FRFMSB                                0x06
#define REG_FRFMID                                0x07
#define REG_FRFLSB                                0x08
// Tx settings
#define REG_PACONFIG                              0x09
#define REG_PARAMP                                0x0A
#define REG_OCP                                   0x0B
// Rx settings
#define REG_LNA                                   0x0C

// LoRa registers
#define REG_FIFOADDRPTR                           0x0D
#define REG_FIFOTXBASEADDR                        0x0E
#define REG_FIFORXBASEADDR                        0x0F
#define REG_FIFORXCURRENTADDR                     0x10
#define REG_IRQFLAGSMASK                          0x11
#define REG_IRQFLAGS                              0x12
#define REG_NBRXBYTES                             0x13
#define REG_RXHEADERCNTVALUEMSB                   0x14
#define REG_RXHEADERCNTVALUELSB                   0x15
#define REG_RXPACKETCNTVALUEMSB                   0x16
#define REG_RXPACKETCNTVALUELSB                   0x17
#define REG_MODEMSTAT                             0x18
#define REG_PKTSNRVALUE                           0x19
#define REG_PKTRSSIVALUE                          0x1A
#define REG_RSSIVALUE                             0x1B
#define REG_HOPCHANNEL                            0x1C
#define REG_MODEMCONFIG1                          0x1D
#define REG_MODEMCONFIG2                          0x1E
#define REG_SYMBTIMEOUTLSB                        0x1F
#define REG_PREAMBLEMSB                           0x20
#define REG_PREAMBLELSB                           0x21
#define REG_PAYLOADLENGTH                         0x22
#define REG_PAYLOADMAXLENGTH                      0x23
#define REG_HOPPERIOD                             0x24
#define REG_FIFORXBYTEADDR                        0x25
#define REG_MODEMCONFIG3                          0x26
#define REG_SYNCWORD							0x39
// end of documented register in datasheet
// I/O settings
#define REG_DIOMAPPING1                           0x40
#define REG_DIOMAPPING2                           0x41
// Version
#define REG_VERSION                               0x42
// Additional settings
#define REG_PLLHOP                                0x44
#define REG_TCXO                                  0x4B
#define REG_PADAC                                 0x4D
#define REG_FORMERTEMP                            0x5B
#define REG_BITRATEFRAC                           0x5D
#define REG_AGCREF                                0x61
#define REG_AGCTHRESH1                            0x62
#define REG_AGCTHRESH2                            0x63
#define REG_AGCTHRESH3                            0x64

#define GPIO_VARE_1                                  0X00
#define GPIO_VARE_2                                  0X00
#define MODEMCONFIG2_SF_MASK                    0x0f
#define MODEMCONFIG1_CODINGRATE_MASK            0xF1
#define MODEMCONFIG2_RXPAYLOADCRC_MASK          0xFB
#define MODEMCONFIG1_BW_MASK                    0x0F
#define MODEMCONFIG1_IMPLICITHEADER_MASK        0xFE
#define MODEMCONFIG2_SYMBTIMEOUTMSB_MASK        0xfc
#define MODEMCONFIG3_LOWDATARATE_OPTI_MASK           0xF7

#define TIME_OUT_INT                                 0x80
#define PACKET_RECVER_INT                            0x40
#define CRC_ERROR_INT                                0x20
#define RECVER_HEAR_INT                              0x10
#define FIFO_SEND_OVER                               0x08
#define IRQFLAGS_CAD                            0x04
#define IRQFLAGS_FHSS                           0x02
#define IRQFLAGS_CADD                           0x01

#define IRQN_TXD_Value                               0xF7
#define IRQN_RXD_Value                               0x9F
#define IRQN_CAD_Value                               0xFA
#define IRQN_SEELP_Value                             0xFF
#define PACKET_MIAX_Value                            0xff

typedef enum {
	SLEEP = (uint8_t)0x00,
	STANDBY = (uint8_t)0x01,
	SYNT_TX = (uint8_t)0x02,
	TX = (uint8_t)0x03,
	SYNT_RX = (uint8_t)0x04,
	RX = (uint8_t)0x05,
	RX_SINGLE = (uint8_t)0x06,
	CAD = (uint8_t)0x07,
} opmode_t;

typedef enum {
	FSK = (uint8_t)0x00,
	LORA = (uint8_t)0x80,
} rfmode_t;

uint8_t sx1278_get_rfmode();
uint8_t sx1278_get_opmode();
uint8_t sx1278_get_syncword();
uint8_t sx1278_get_spread_fact();
void sx1278_get_rf_freq(uint8_t *d);
uint16_t sx1278_get_preamble_len();
int sx1278_get_rssi();

void sx1278_set_rf_freq(void);
void sx1278_set_rf_power(uint8_t power);
void sx1278_set_spread_fact(uint8_t factor);
void sx1278_set_error_coding(uint8_t value);
void sx1278_set_crc_on(bool enable);
void sx1278_set_bandwidth(uint8_t bw);
void sx1278_set_head_on(bool enable);
void sx1278_set_payload_len(uint8_t value);
void sx1278_set_symb_timeout(unsigned int value);
void sx1278_set_nb_trig_peaks(uint8_t value);
void sx1278_set_syncword(uint8_t sw);

void sx1278_init(void);
void sx1278_set_opmode(opmode_t op);
void SX1278_set_rfmode(rfmode_t op);
void sx1278_set_rf_power(uint8_t power);

void sx1278_send_data(uint8_t *data, uint8_t len);

#endif
