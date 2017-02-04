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
#ifndef __APP_H__
#define __APP_H__

typedef struct ralay_status_t {
	uint8_t r1;
	uint8_t r2;
	uint8_t r3;
	uint8_t r4;
	uint8_t r5;
	uint8_t r6;
	uint8_t pack[2];
} __attribute__((aligned(4), packed)) relay_status_t;

typedef struct ctrl_status_t {
	uint8_t init_flag;
	uint32_t start_count;
	relay_status_t relay_status;
	uint8 packed[3];
	char ch1_voice_name[32];
	char ch2_voice_name[32];
	char ch3_voice_name[32];
	char ch4_voice_name[32];
	char ch5_voice_name[32];
	char ch6_voice_name[32];
} __attribute__((aligned(4), packed)) ctrl_status_t;

void param_load();
void param_save(void);
void app_push_status(relay_status_t *st);
void app_check_set_push_save(relay_status_t *st);

#endif
