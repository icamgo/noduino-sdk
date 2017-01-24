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
#include "osapi.h"
#include "user_interface.h"
#include "conn.h"

#define SSID "YOUR_WIFI_SSID"
#define PASSWORD "YOUR_WIFI_PASSWD"

/*
 * Note: check_memleak_debug_enable is a weak function inside SDK.
 * please copy following codes to user_main.c
 */
#if 0
#include "mem.h"

bool ICACHE_FLASH_ATTR check_memleak_debug_enable(void)
{
	return MEMLEAK_DEBUG_ENABLE;
}
#endif

LOCAL void ICACHE_FLASH_ATTR wifi_handle_event_cb(System_Event_t * evt)
{
	INFO("event %x\n", evt->event);

	switch (evt->event) {
	case EVENT_STAMODE_CONNECTED:
		break;
	case EVENT_STAMODE_DISCONNECTED:
		INFO("disconnect from ssid %s, reason %d\n",
			evt->event_info.disconnected.ssid,
			evt->event_info.disconnected.reason);
		break;
	case EVENT_STAMODE_AUTHMODE_CHANGE:
		INFO("mode: %d -> %d\n",
			evt->event_info.auth_change.old_mode,
			evt->event_info.auth_change.new_mode);
		break;
	case EVENT_STAMODE_GOT_IP:
		os_printf("heap %d\n", system_get_free_heap_size());
		user_conn_init();
		break;
	case EVENT_SOFTAPMODE_STACONNECTED:
		INFO("station: " MACSTR "join, AID = %d\n",
			MAC2STR(evt->event_info.sta_connected.mac),
			evt->event_info.sta_connected.aid);
		break;
	case EVENT_SOFTAPMODE_STADISCONNECTED:
		INFO("station: " MACSTR "leave, AID = %d\n",
			MAC2STR(evt->event_info.sta_disconnected.mac),
			evt->event_info.sta_disconnected.aid);
		break;
	default:
		break;
	}
}

void ICACHE_FLASH_ATTR user_set_station_config(void)
{
	// Wifi configuration 
	char ssid[32] = SSID;
	char password[64] = PASSWORD;
	struct station_config stationConf;

	os_memset(stationConf.ssid, 0, 32);
	os_memset(stationConf.password, 0, 64);
	//need not mac address
	stationConf.bssid_set = 0;

	//Set ap settings 
	memcpy(&stationConf.ssid, ssid, 32);
	memcpy(&stationConf.password, password, 64);
	wifi_station_set_config(&stationConf);
}

void ICACHE_FLASH_ATTR user_init(void)
{
	INFO("SDK version:%s\n", system_get_sdk_version());
	system_print_meminfo();
	wifi_set_event_handler_cb(wifi_handle_event_cb);
	wifi_set_opmode(STATION_MODE);
	user_set_station_config();
	wifi_set_sleep_type(NONE_SLEEP_T);
}
