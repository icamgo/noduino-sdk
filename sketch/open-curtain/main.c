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
#include "compile.h"

os_timer_t online_timer;

uint32_t check_online_sec = 300;	// 3 min

static void mjyun_stated_cb(mjyun_state_t state)
{
    if (mjyun_state() != state)
        INFO("Platform: mjyun_state error \r\n");

    switch (state)
    {
        case WIFI_IDLE:
            INFO("Platform: WIFI_IDLE\r\n");
            break;
        case WIFI_SMARTLINK_LINKING:
            INFO("Platform: WIFI_SMARTLINK_LINKING\r\n");
            break;
        case WIFI_SMARTLINK_FINDING:
            INFO("Platform: WIFI_SMARTLINK_FINDING\r\n");
            break;
        case WIFI_SMARTLINK_TIMEOUT:
            INFO("Platform: WIFI_SMARTLINK_TIMEOUT\r\n");
            break;
        case WIFI_SMARTLINK_GETTING:
            INFO("Platform: WIFI_SMARTLINK_GETTING\r\n");
            break;
        case WIFI_SMARTLINK_OK:
            INFO("Platform: WIFI_SMARTLINK_OK\r\n");
            break;
        case WIFI_AP_OK:
            INFO("Platform: WIFI_AP_OK\r\n");
            break;
        case WIFI_AP_ERROR:
            INFO("Platform: WIFI_AP_ERROR\r\n");
            break;
        case WIFI_AP_STATION_OK:
            INFO("Platform: WIFI_AP_STATION_OK\r\n");
            break;
        case WIFI_AP_STATION_ERROR:
            INFO("Platform: WIFI_AP_STATION_ERROR\r\n");
            break;
        case WIFI_STATION_OK:
            INFO("Platform: WIFI_STATION_OK\r\n");
            break;
        case WIFI_STATION_ERROR:
            INFO("Platform: WIFI_STATION_ERROR\r\n");
            break;
        case MJYUN_CONNECTING:
            INFO("Platform: MJYUN_CONNECTING\r\n");
            break;
        case MJYUN_CONNECTING_ERROR:
            INFO("Platform: MJYUN_CONNECTING_ERROR\r\n");
            break;
        case MJYUN_CONNECTED:
            INFO("Platform: MJYUN_CONNECTED \r\n");
            break;
        case MJYUN_DISCONNECTED:
            INFO("Platform: MJYUN_DISCONNECTED\r\n");
            break;
        default:
            break;
	}
}

void mjyun_receive(const char *event_name, const char *event_data)
{
	INFO("RECEIVED: key:value [%s]:[%s]\r\n", event_name, event_data);

	if(os_strncmp(event_name, "set", 3) == 0) {
		cJSON * pD = cJSON_Parse(event_data);
		INFO("curtain set event!\r\n");

		if ((NULL != pD) && (cJSON_Object == pD->type)) {
			cJSON * pS = cJSON_GetObjectItem(pD, "status");
			cJSON * pP = cJSON_GetObjectItem(pD, "position");

			if ((NULL != pS) && (cJSON_Number == pS->type) &&
				(NULL != pP) && (cJSON_Number == pP->type)) {
				int st = pS->valueint;
				int pos = pP->valueint;

				INFO("rx mqtt, set curtain\r\n");
				curtain_set_status_and_publish(st, pos);
			}
		} else {
			INFO("%s: Invalid JSON format!\r\n", __func__);
		}

		cJSON_Delete(pD);
	}

	if(os_strncmp(event_data, "get", 3) == 0) {
		INFO("Received get status command!\r\n");
		curtain_publish_status();
	}

	if(os_strncmp(event_data, "ota", 3) == 0) {
		INFO("OTA: upgrade the firmware!\r\n");
		mjyun_mini_ota_start("ota/dev/opencurtain/files");
	}

	if(os_strncmp(event_data, "reset", 5) == 0) {
		INFO("Received Reset system command, reset now!\r\n");
		system_restart();
	}
}

#ifdef CONFIG_MQTT_ROBUST
static upload_fail_cnt = 0;

void http_error_handle()
{
	upload_fail_cnt++;
	if(upload_fail_cnt >= 3) {
		// failed about 5min
		os_printf("http pushed failed %d times, reset the system\r\n", upload_fail_cnt);
		//system_restart();
	}

	//TODO: store the data in flash
}

void check_online_cb(char *response, int http_status, char *full_response)
{
	if(HTTP_STATUS_GENERIC_ERROR != http_status) {

		INFO( "%s: response=%s<EOF>\r\n", __func__, response );
		INFO( "%s: memory left=%d\r\n", __func__, system_get_free_heap_size() );

		cJSON* pRoot = cJSON_Parse(response);
		if((NULL != pRoot) && (cJSON_Object == pRoot->type)) {
			cJSON * pOnline = cJSON_GetObjectItem(pRoot, "online");

			if ((NULL != pOnline) && (cJSON_Number == pOnline->type)) {
				if (0 == pOnline->valueint) {
					//need to restart the system
					INFO("Using http to find device offline, reset the device\r\n");
					system_restart();
				} else if (1 == pOnline->valueint) {
					INFO("Device online (via http)\r\n");
				}
			}
		} else  {
			INFO( "%s: Error when parse JSON\r\n", __func__ );
		}
		cJSON_Delete(pRoot);

	} else {
		http_error_handle();
		INFO("%s: http_status=%d\r\n", __func__, http_status);
	}
}

void check_online()
{
	uint8_t *buf = (uint8_t *) os_zalloc(os_strlen(HTTP_CHECK_ONLINE_URL) +
					os_strlen(mjyun_getdeviceid()));
	if (buf == NULL) {
		INFO( "%s: not enough memory\r\n", __func__ );
		return;
	}

	os_sprintf(buf, HTTP_CHECK_ONLINE_URL, mjyun_getdeviceid());

	http_get((const char *)buf, "Content-Type:application/json\r\n", check_online_cb);
	INFO("%s\r\n", (char *)buf);
	os_free(buf);
}
#endif

void mjyun_connected()
{
	curtain_publish_status();

#ifdef CONFIG_MQTT_ROBUST
	os_timer_disarm(&online_timer);
	os_timer_setfn(&online_timer, (os_timer_func_t *) check_online, NULL);
	os_timer_arm(&online_timer, check_online_sec*1000, 1);
#endif

	// stop to show the wifi status
	wifi_led_disable();
}

void mjyun_disconnected()
{
	// show the wifi status
	wifi_led_enable();
}

const mjyun_config_t mjyun_conf = {
	"MJP4760972804",		/* MK Noduino Curtian KT320, use WiFi Ctrl of WXMP */
	HW_VERSION,
	FW_VERSION,
	FW_VERSION,				/* 设备上线时，给app发送 online 消息中的附加数据，[选填] */
	"Device Offline",		/* 设备掉线时，给app发送 offline 消息中的附加数据，[选填] */
	WITH_MQTT
};

irom void init_yun()
{
	mjyun_statechanged(mjyun_stated_cb);
	mjyun_ondata(mjyun_receive);
	mjyun_onconnected(mjyun_connected);
	mjyun_ondisconnected(mjyun_disconnected);

	mjyun_run(&mjyun_conf);
	wifi_set_sleep_type(MODEM_SLEEP_T);
}

irom void user_init(void)
{
#ifdef DEBUG
	uart_init(115200, 115200);
#endif
	os_delay_us(100);
	INFO("\r\n\r\n\r\n\r\n\r\n\r\n");
	INFO("\r\nWelcom to Noduino Open Curtain!\r\n");
	INFO("Current firmware is user%d.bin\r\n", system_upgrade_userbin_check()+1);
	INFO("%s", noduino_banner);

	noduino_init();

	param_init();
	curtain_init();
	encoder_init();

	//curtain_set_status(param_get_status(), param_get_position());

	init_yun();
}
