/*
 *  Copyright (c) 2016 - 2026 MaiKe Labs
 *
 *  Library for MY9291 LED driver Chip
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
#ifndef __MY9291_H__
#define __MY9291_H__
    
#include "noduino.h"
    
#ifdef DEBUG 
#define INFO(format, ...) os_printf(format, ##__VA_ARGS__)
#else	/*  */
#define INFO(format, ...)
#endif	/*  */

typedef enum my9291_cmd_one_shot_t {
	MY9291_CMD_ONE_SHOT_DISABLE = 0X00,
	MY9291_CMD_ONE_SHOT_ENFORCE = 0X01, 
} my9291_cmd_one_shot_t;

typedef enum my9291_cmd_reaction_t {
	MY9291_CMD_REACTION_FAST = 0X00,
	MY9291_CMD_REACTION_SLOW = 0X01, 
} my9291_cmd_reaction_t;

typedef enum my9291_cmd_bit_width_t {
	MY9291_CMD_BIT_WIDTH_16 = 0X00,
	MY9291_CMD_BIT_WIDTH_14 = 0X01,
	MY9291_CMD_BIT_WIDTH_12 = 0X02,
	MY9291_CMD_BIT_WIDTH_8 = 0X03, 
} my9291_cmd_bit_width_t;

typedef enum my9291_cmd_frequency_t {
	MY9291_CMD_FREQUENCY_DIVIDE_1 = 0X00,
	MY9291_CMD_FREQUENCY_DIVIDE_4 = 0X01,
	MY9291_CMD_FREQUENCY_DIVIDE_16 = 0X02,
	MY9291_CMD_FREQUENCY_DIVIDE_64 = 0X03, 
} my9291_cmd_frequency_t;

typedef enum my9291_cmd_scatter_t {
	MY9291_CMD_SCATTER_APDM = 0X00,
	MY9291_CMD_SCATTER_PWM = 0X01, 
} my9291_cmd_scatter_t;

typedef struct my9291_cmd_t {
	my9291_cmd_scatter_t scatter:1;
	my9291_cmd_frequency_t frequency:2;
	my9291_cmd_bit_width_t bit_width:2;
	my9291_cmd_reaction_t reaction:1;
	my9291_cmd_one_shot_t one_shot:1;
	uint8_t resv:1;
} __attribute__ ((aligned(1), packed)) my9291_cmd_t;
 
#define MY9291_COMMAND_DEFAULT 					\
{												\
	.scatter = MY9291_cmd_SCATTER_APDM,			\
	.frequency = MY9291_CMD_FREQUENCY_DIVIDE_1,	\
	.bit_width = MY9291_CMD_BIT_WIDTH_8,,		\
	.reaction = MY9291_CMD_REACTION_FAST,		\
	.one_shot = MY9291_CMD_ONE_SHOT_DISABLE,	\
	.resv = 0,									\
}

void my9291_init(uint8_t pin_di, uint8_t pin_dcki,
				       my9291_cmd_t command);
void my9291_di_pulse(uint16_t times);
void my9291_dcki_pulse(uint16_t times);
void my9291_send_duty(uint16_t duty_r, uint16_t duty_g, uint16_t duty_b,
		       uint16_t duty_w);
 
#endif	/* __NETWORK_H__ */
