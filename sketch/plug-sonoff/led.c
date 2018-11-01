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
#include "user_config.h"

irom void led_init()
{
	// GPIO13: the wifi status led
	wifi_status_led_install(13, PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);
}

irom void wifi_led_enable()
{
	// GPIO13: the wifi status led
	wifi_status_led_install(13, PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);
}

irom void wifi_led_disable()
{
	wifi_status_led_uninstall();
}
