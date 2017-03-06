/*
 *  Copyright (c) 2016 - 2025 MaiKe Labs
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
#ifndef __UPNP_H__
#define __UPNP_H__

#include "udp_srv.h"

typedef enum {
	WEMO_SWITCH,
	WEMO_INSIGHT,
	WEMO_LIGHTSWITCH,
	WEMO_BULB,
	WEMO_MOTION,
	WEMO_MAKER,
	HUE
} dev_type_t;

typedef void (*on_fn_t)(void);
typedef void (*off_fn_t)(void);
typedef void (*pos_fn_t)(int p);
typedef int (*get_pos_t)();
typedef bool (*get_on_t)();

typedef struct upnp_dev {
	struct espconn *esp_conn;
	uint32_t port;
	char dev_upnp_uuid[64];
	char dev_voice_name[32];
	char model_name[32];
	char model_num[12];
	dev_type_t dev_type;
	on_fn_t way_on;
	off_fn_t way_off;
	pos_fn_t set_pos;
	get_pos_t get_pos;
	get_on_t get_on;
} upnp_dev_t;

char *get_dev_target(dev_type_t tp);

int upnp_start(upnp_dev_t *devs, int ways);
void upnp_stop(upnp_dev_t *devs, int ways);
int upnp_ssdp_start();
void upnp_ssdp_stop();
#endif
