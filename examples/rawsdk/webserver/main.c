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

irom void user_check_ip(void)
{
	struct ip_info ipconfig;

	//disarm timer first
	os_timer_disarm(&test_timer);

	//get ip info of ESP8266 station
	wifi_get_ip_info(STATION_IF, &ipconfig);

	if (wifi_station_get_connect_status() == STATION_GOT_IP
	    && ipconfig.ip.addr != 0) {

		os_printf("Got ip ! \r\n");

		httpd_start(&httpd);
	} else {

		if ((wifi_station_get_connect_status() == STATION_WRONG_PASSWORD
		     || wifi_station_get_connect_status() == STATION_NO_AP_FOUND
		     || wifi_station_get_connect_status() ==
		     STATION_CONNECT_FAIL)) {

			os_printf("Connect fail ! \r\n");

		} else {

			//re-arm timer to check ip
			os_timer_setfn(&test_timer, (os_timer_func_t *)
				       user_check_ip, NULL);
			os_timer_arm(&test_timer, 100, 0);
		}
	}
}

irom void start_station_mode(void)
{
	// Wifi configuration
	char ssid[32] = SSID;
	char password[64] = PASSWORD;
	struct station_config conf;

	//Set station mode
	wifi_set_opmode(STATION_MODE);

	//need not mac address
	conf.bssid_set = 0;

	//Set ap settings
	os_memcpy(&conf.ssid, ssid, 32);
	os_memcpy(&conf.password, password, 64);
	wifi_station_set_config(&conf);

	//set a timer to check whether got ip from router succeed or not.
	os_timer_disarm(&test_timer);
	os_timer_setfn(&test_timer,
		       (os_timer_func_t *) user_check_ip, NULL);
	os_timer_arm(&test_timer, 100, 0);

}

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

	start_station_mode();
	//start_ap_mode();
}
