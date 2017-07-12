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
#include "user_config.h"

static os_timer_t led_ef_timer;

static uint8_t smart_ef = 0;

void led_effect_tick()
{
	static bool val = 0;

	os_timer_disarm(&led_ef_timer);

	switch (smart_ef) {
		case 0:
			led_set(val);
			val = !val;
			os_timer_setfn(&led_ef_timer, (os_timer_func_t *)led_effect_tick, NULL);
			os_timer_arm(&led_ef_timer, 800, 1);
			break;
		case 1:
			led_set(0);
			return;
			break;
	}
}

void led_set_effect(uint8_t ef)
{
	smart_ef = ef;
	led_effect_tick();
}

irom void led_init()
{
	pinMode(LED_GPIO_NUM, OUTPUT);

	// LED_GPIO_NUM: the wifi status led
	wifi_status_led_install(LED_GPIO_NUM, LED_GPIO_MUX,
			LED_GPIO_FUNC);
}

irom void wifi_led_enable()
{
	// LED_GPIO_NUM: the wifi status led
	wifi_status_led_install(LED_GPIO_NUM, LED_GPIO_MUX,
			LED_GPIO_FUNC);
}

irom void wifi_led_disable()
{
	wifi_status_led_uninstall();
}

void led_set(uint8_t st)
{
	digitalWrite(LED_GPIO_NUM, st);
}

void led_on()
{
	led_set(1);
}

void led_off()
{
	led_set(0);
}
