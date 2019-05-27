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
#include "user_config.h"

static int led_status = 0;

irom void led_init()
{
	led_off();

	// GPIO3: the wifi status led
	wifi_status_led_install (3, PERIPHS_IO_MUX_U0RXD_U, FUNC_GPIO3);
}

irom void wifi_led_enable()
{
	// GPIO13: the wifi status led
	wifi_status_led_install (3, PERIPHS_IO_MUX_U0RXD_U, FUNC_GPIO3);
}

irom void wifi_led_disable()
{
	wifi_status_led_uninstall();
}

irom void led_on()
{
	INFO("led on\r\n");
	INFO("set gpio3 to high\r\n");
	gpio_output_set(BIT3, 0, BIT3, 0);
	led_status = 1;
}

irom void led_off()
{
	INFO("led off\r\n");
	INFO("set gpio3 to low\r\n");
	gpio_output_set(0, BIT3, BIT3, 0);
	led_status = 0;
}

irom void toggle_led()
{
	if(led_status == 0)
		led_on();
	else
		led_off();
}
