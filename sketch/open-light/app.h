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

// FLASH MAP
// 0x79: SAVE1 0x7A: SAVE2 0x7B: FLAG
#define APP_START_SEC   	0x79
#define APP_POWER_PERCENT 	100
#define APP_POWER_LIMIT(x) 	((x) * (APP_POWER_PERCENT) / 100)

typedef struct mcu_status_t {
	uint8 s;
	uint8 r;
	uint8 g;
	uint8 b;
	uint8 w;
} __attribute__((aligned(1), packed)) mcu_status_t;

typedef struct hsl {
	float h;
	float s;
	float l;
} __attribute__((aligned(4), packed)) hsl_t;

typedef struct system_status_t {
	uint8 init_flag;
	uint16 start_count;
	uint8 start_continue;
	mcu_status_t mcu_status;
	uint8 sc_effect;
	char voice_name[32];
	uint8 cold_on;
	uint8 grad_on;
	uint8 alexa_on;
	uint8 airkiss_nff_on;
	uint8 rgbw_bind;
	uint8 pack[1];
} __attribute__((aligned(4), packed)) system_status_t;

typedef enum app_state_t {
	APP_STATE_NORMAL,
	APP_STATE_SMART,
	APP_STATE_UPGRADE,
	APP_STATE_RESTORE,
} app_state_t;

typedef enum light_effect {
	NONE_EFFECT = 0,
	RED_GRADIENT,
	GREEN_GRADIENT,
	BLUE_GRADIENT,
	WHITE_GRADIENT,
} light_effect_t;

void mjyun_receive(const char * event_name, const char * event_data);

void set_light_effect(light_effect_t e);
void set_light_status(mcu_status_t *pst);
void change_light_grad(mcu_status_t *to);
void change_light_lum(int bri);
int get_light_lum();
bool get_light_on();

void app_param_load(void);
void app_param_save(void);
void app_param_restore(void);
bool is_warm_boot();
void set_warm_boot_flag();

void app_push_status(mcu_status_t *pst);
void app_push_voice_name(char *vname);
void app_push_cold_on();
void app_push_grad_on();
void app_push_alexa_on();
void app_push_airkiss_nff_on();

void app_check_mcu_save(mcu_status_t *st);
void app_start_check(uint32_t system_start_seconds);

#endif
