/*
 *  Copyright (c) 2015 - 2025 MaiKe Labs
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
#ifndef	__MCP342X_H__
#define	__MCP342X_H__

#include "twi.h"

//#define	MCP342X_DEBUG		1

#ifdef MCP342X_DEBUG
#define MCP342X_INFO( format, ... ) os_printf( format, ## __VA_ARGS__ )
#else
#define MCP342X_INFO( format, ... )
#endif

// I2C address for MCP3422 - base address for MCP3424 = 0x68
#define MCP342X_ADDRESS 0X68

// fields in configuration register
#define MCP342X_GAIN_FIELD 0X03 // PGA field
#define MCP342X_GAIN_X1    0X00 // PGA gain X1, 12 bits, 1 mV
#define MCP342X_GAIN_X2    0X01 // PGA gain X2, 14 bits, 250 uV
#define MCP342X_GAIN_X4    0X02 // PGA gain X4, 16 bits, 62.5 uV
#define MCP342X_GAIN_X8    0X03 // PGA gain X8, 18 bits, 15.625 uV

#define MCP342X_RES_FIELD  0X0C // resolution/rate field
#define MCP342X_RES_SHIFT  0X02 // shift to low bits
#define MCP342X_12_BIT     0X00 // 12-bit 240 SPS
#define MCP342X_14_BIT     0X04 // 14-bit 60 SPS
#define MCP342X_16_BIT     0X08 // 16-bit 15 SPS
#define MCP342X_18_BIT     0X0C // 18-bit 3.75 SPS

#define MCP342X_CONTINUOUS 0X10 // 1 = continuous, 0 = one-shot

#define MCP342X_CHAN_FIELD 0X60 // channel field
#define MCP342X_CHANNEL_1  0X00 // select MUX channel 1
#define MCP342X_CHANNEL_2  0X20 // select MUX channel 2
#define MCP342X_CHANNEL_3  0X40 // select MUX channel 3
#define MCP342X_CHANNEL_4  0X60 // select MUX channel 4

#define MCP342X_START      0X80 // write: start a conversion
#define MCP342X_BUSY       0X80 // read: output not ready

// default adc configuration register - resolution and gain added in setup()
#define	MCP342X_ADC_CFG		(MCP342X_START			\
							| MCP342X_CHANNEL_1		\
							| MCP342X_CONTINUOUS	\
							| MCP342X_GAIN_X1		\
							| MCP342X_18_BIT)


void mcp342x_init();
int mcp342x_set_cfg(uint8_t cfg);
void mcp342x_set_oneshot();
void mcp342x_set_continuous();
int mcp342x_write(uint8_t data);

int mcp342x_get_uv();

#endif
