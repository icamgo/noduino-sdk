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
#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#include "ets_sys.h"
#include "osapi.h"
#include "user_interface.h"
#include "os_type.h"
#include "mem.h"
#include "gpio.h"

#include "driver/key.h"
#include "driver/uart.h"
#include "noduino.h"

#include "upnp.h"

#include "mjyun.h"

#define CONFIG_ALEXA		1
#define DEFAULT_VOICE_NAME	"open plug"

#define DEBUG				1

#ifdef DEBUG
#define INFO( format, ... ) os_printf( format, ## __VA_ARGS__ )
#else
#define INFO( format, ... )
#endif

#define LED_GPIO_NUM		3
#define KEY_GPIO_NUM		0
#define RELAY_GPIO_NUM		12

#if (LED_GPIO_NUM == 3)
#define LED_GPIO_MUX		PERIPHS_IO_MUX_U0RXD_U
#define LED_GPIO_FUNC		FUNC_GPIO3
#elif (LED_GPIO_NUM == 12)
#define LED_GPIO_MUX		PERIPHS_IO_MUX_MTDI_U
#define LED_GPIO_FUNC		FUNC_GPIO12
#elif (LED_GPIO_NUM == 13)
#define LED_GPIO_MUX		PERIPHS_IO_MUX_MTCK_U
#define LED_GPIO_FUNC		FUNC_GPIO13
#elif (LED_GPIO_NUM == 14)
#define LED_GPIO_MUX		PERIPHS_IO_MUX_MTMS_U
#define LED_GPIO_FUNC		FUNC_GPIO14
#elif (LED_GPIO_NUM == 15)
#define LED_GPIO_MUX		PERIPHS_IO_MUX_MTDO_U
#define LED_GPIO_FUNC		FUNC_GPIO15
#endif


#define	KEY_NUM				1

#if (KEY_GPIO_NUM == 0)
#define KEY_GPIO_MUX		PERIPHS_IO_MUX_GPIO0_U
#define KEY_GPIO_FUNC		FUNC_GPIO0
#elif (KEY_GPIO_NUM == 12)
#define KEY_GPIO_MUX		PERIPHS_IO_MUX_MTDI_U
#define KEY_GPIO_FUNC		FUNC_GPIO12
#elif (KEY_GPIO_NUM == 13)
#define KEY_GPIO_MUX		PERIPHS_IO_MUX_MTCK_U
#define KEY_GPIO_FUNC		FUNC_GPIO13
#elif (KEY_GPIO_NUM == 1)
#define KEY_GPIO_MUX		PERIPHS_IO_MUX_U0TXD_U
#define KEY_GPIO_FUNC		FUNC_GPIO1
#endif

// NOTICE: --- For 1024KB spi flash
// 0xFA000
#define PARAM_START_SEC		0xFA

struct dev_param {
	uint8_t status;
	uint8_t pad[3];
	char voice_name[32];
	uint8_t cold_on;
	uint8_t alexa_on;
	uint8_t airkiss_nff_on;
	uint8_t pack[1];
} __attribute__((aligned(4), packed));


void xkey_init();

void relay_on();
void relay_off();
void relay_init();
void relay_set_status(uint8_t status);
void relay_set_status_and_publish(uint8_t status);
uint8_t relay_get_status();
void relay_publish_status();

void param_set_status(uint8_t status);
uint8_t param_get_status(void);
void param_save(void);
void param_restore(void);
void param_init();

void led_init();
void wifi_led_enable();
void wifi_led_disable();
void led_set_effect(uint8_t ef);
void led_set(uint8_t st);
void led_on();
void led_off();

#endif
