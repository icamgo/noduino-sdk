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

static os_timer_t test_timer;

static httpd_t httpd = {
   .esp_conn = NULL,
   .port = 80
};

irom void start_ap_mode()
{
	struct softap_config config;
	char ssid[] = "Noduino";

	// switch to softp ap mode
	wifi_set_opmode(SOFTAP_MODE);

	// get old softap config
	wifi_softap_get_config(&config);

	os_memcpy(config.ssid, ssid, os_strlen(ssid));
	config.ssid_len = os_strlen(ssid);

	// set the new ssid
	wifi_softap_set_config(&config);
}

void user_init(void)
{
	uart_init(115200, 115200);
	os_printf("SDK version:%s\n", system_get_sdk_version());

	start_ap_mode();
	httpd_start(&httpd);
}
