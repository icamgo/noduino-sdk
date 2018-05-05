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
#include "time.h"
#include "sht2x.h"
#include "compile.h"

#define DEBUG				1

static uint32_t realtime = 0;
static uint32_t network_state = 0;
static uint32_t wan_ok = 0;

irom static void mjyun_stated_cb(mjyun_state_t state)
{
    if (mjyun_state() != state)
        os_printf("Platform: mjyun_state error \r\n");

    switch (state)
    {
        case WIFI_IDLE:
            os_printf("Platform: WIFI_IDLE\r\n");
            break;
        case WIFI_SMARTLINK_LINKING:
            os_printf("Platform: WIFI_SMARTLINK_LINKING\r\n");
            break;
        case WIFI_SMARTLINK_FINDING:
            os_printf("Platform: WIFI_SMARTLINK_FINDING\r\n");
            break;
        case WIFI_SMARTLINK_TIMEOUT:
            os_printf("Platform: WIFI_SMARTLINK_TIMEOUT\r\n");
            break;
        case WIFI_SMARTLINK_GETTING:
            os_printf("Platform: WIFI_SMARTLINK_GETTING\r\n");
            break;
        case WIFI_SMARTLINK_OK:
            os_printf("Platform: WIFI_SMARTLINK_OK\r\n");
            break;
        case WIFI_AP_OK:
            os_printf("Platform: WIFI_AP_OK\r\n");
            break;
        case WIFI_AP_ERROR:
            os_printf("Platform: WIFI_AP_ERROR\r\n");
            break;
        case WIFI_AP_STATION_OK:
            os_printf("Platform: WIFI_AP_STATION_OK\r\n");
			network_state = WIFI_AP_STATION_OK;
            break;
        case WIFI_AP_STATION_ERROR:
            os_printf("Platform: WIFI_AP_STATION_ERROR\r\n");
            break;
        case WIFI_STATION_OK:
            os_printf("Platform: WIFI_STATION_OK\r\n");
			network_state = WIFI_STATION_OK;
            break;
        case WIFI_STATION_ERROR:
            os_printf("Platform: WIFI_STATION_ERROR\r\n");
            break;
        case WIFI_STA_DISCONNECTED:
            os_printf("Platform: WIFI_STA_DISCONNECTED\r\n");
			network_state = WIFI_STA_DISCONNECTED;
            break;
        case MJYUN_CONNECTING:
            os_printf("Platform: MJYUN_CONNECTING\r\n");
            break;
        case MJYUN_CONNECTING_ERROR:
			wan_ok = 0;
            os_printf("Platform: MJYUN_CONNECTING_ERROR\r\n");
            break;
        case MJYUN_CONNECTED:
			wan_ok = 1;
            os_printf("Platform: MJYUN_CONNECTED \r\n");
            break;
        case MJYUN_DISCONNECTED:
			wan_ok = 0;
            os_printf("Platform: MJYUN_DISCONNECTED\r\n");
            break;
        default:
            break;
	}
}

irom void mjyun_receive(const char *event_name, const char *event_data)
{
#ifdef DEBUG
	os_printf("RECEIVED: key:value [%s]:[%s]", event_name, event_data);
#endif

	if(os_strncmp(event_data, "ota", 3) == 0)
	{
#ifdef DEBUG
		os_printf("OTA: upgrade the firmware!\r\n");
#endif
		mjyun_mini_ota_start("ota/dev/openikair/files");
	}
}

os_timer_t time_temp;

static int mqttrate_sec = MQTT_SEND_RATE_SEC; //2 second
static int httprate_min = HTTP_SEND_RATE_MIN; //5 min

static int tmp_timer = 0;

irom char *strstrip(char *s)
{
	size_t size;
	char *end;

	size = strlen(s);

	if (!size)
		return s;

	end = s + size - 1;
	while (end >= s && isspace(*end))
		end--;
	*(end + 1) = '\0';

	while (*s && isspace(*s))
		s++;

	return s;
}

static upload_fail_cnt = 0;

void http_upload_error_handle()
{
	upload_fail_cnt++;
	if(upload_fail_cnt >= 3) {
		// failed about 5min
		os_printf("http pushed failed %d times, reset the system\r\n", upload_fail_cnt);
		//system_restart();
	}

	//TODO: store the data in flash
}

void http_upload_cb(char *response, int http_status, char *full_response)
{
	if(HTTP_STATUS_GENERIC_ERROR != http_status) {
#ifdef DEBUG
		os_printf( "%s: strlen(full_response)=%d\r\n", __func__, strlen( full_response ) );
		os_printf( "%s: response=%s<EOF>\r\n", __func__, response );
		os_printf( "%s: memory left=%d\r\n", __func__, system_get_free_heap_size() );
#endif

		if(os_strncmp(response, "ok", 2) != 0)
			http_upload_error_handle();

	} else {
		http_upload_error_handle();
#ifdef DEBUG
		os_printf( "%s: http_status=%d\r\n", __func__, http_status );
#endif
	}
}

void http_upload(char *tt, char *hh)
{
	uint8_t * URL = (uint8_t *) os_zalloc(os_strlen(HTTP_UPLOAD_URL) +
	                  os_strlen(mjyun_getdeviceid()) +
	                  os_strlen(tt) + os_strlen(hh) +
	                  12 + 12);

	if ( URL == NULL ) {
#ifdef DEBUG
		os_printf( "%s: not enough memory\r\n", __func__ );
#endif
		return;
	}

	uint8_t ma[6];
	char sta_mac[16];
	os_memset(sta_mac, 0, sizeof(sta_mac) / sizeof(char));
	wifi_get_macaddr(STATION_IF, ma);
	os_sprintf(sta_mac, "%02X%02X%02X%02X%02X%02X", ma[0], ma[1], ma[2], ma[3], ma[4], ma[5]);

	uint32_t cs = time(NULL);

	os_sprintf(URL,
	           HTTP_UPLOAD_URL,
	           mjyun_getdeviceid(),
	           (tt),
	           (hh),
	           cs,
	           sta_mac);
	http_post((const char *)URL , "Content-Type:application/json\r\n", "", http_upload_cb);
#ifdef DEBUG
	os_printf("%s\r\n", (char *)URL);
#endif
	os_free( URL );
}

static float pre_temp = 0;
void push_temp_humi()
{
	char t_buf[8];
	char h_buf[8];
	char msg[64];

	os_memset(msg, 0, 64);

	sht2x_reset();
	float temp = sht2x_GetTemperature();
	float humi = sht2x_GetHumidity();
	dtostrf(temp, 5, 1, t_buf),
	dtostrf(humi, 5, 1, h_buf);

	char *t = strstrip(t_buf);
	char *h = strstrip(h_buf);

	//os_printf("Temperature(C): %s\r\n", t);
	//os_printf("Humidity(%RH): %s\r\n", h);

	if (1 == realtime) {
		if (fabsf(temp - pre_temp) > 0.1) {

			pre_temp = temp;

			os_sprintf(msg, "{\"temp\":%s,\"humi\":%s}", t, h);

			/* data changed, need update via mqtt */
			mjyun_publishstatus(msg);
			http_upload(t, h);
		}
	} else {
		http_upload(t, h);
	}
}

irom void time_init()
{
	//struct tm *tblock;
	uint32_t cs = time(NULL);
	os_printf("Current timestamp: %d\r\n", cs);
}

void mjyun_connected()
{
	time_init();

	mjyun_setssidprefix("NOD_");

	wan_ok = 1;

	// stop to show the wifi status
	wifi_led_disable();
}

void mjyun_disconnected()
{
	// show the wifi status
	wifi_led_enable();
}

mjyun_config_t mjyun_conf = {
	"MJP2090591473",		/* Maike Noduino iKair */
	HW_VERSION,				/* 产品子id (一般用于微信设备) [选填]*/
	FW_VERSION,
	FW_VERSION,				/* 设备上线时，给app发送 online 消息中的附加数据，[选填] */
	"Device Offline",		/* 设备掉线时，给app发送 offline 消息中的附加数据，[选填] */
#ifdef LOW_POWER
	0,
#else
	WITH_MQTT,
#endif
};

irom void init_yun()
{
	mjyun_statechanged(mjyun_stated_cb);
	mjyun_ondata(mjyun_receive);
	mjyun_onconnected(mjyun_connected);
	mjyun_ondisconnected(mjyun_disconnected);

	if (realtime == 1)
		mjyun_conf.run_flag |= WITH_MQTT;
	mjyun_run(&mjyun_conf);
}

irom void setup()
{
#ifdef DEBUG
	uart_init(115200, 115200);
#endif
	os_printf("Current firmware is user%d.bin\r\n", system_upgrade_userbin_check()+1);
	os_printf("%s", noduino_banner);

	realtime = 1;

	led_init();
	init_yun();

	wifi_set_sleep_type(MODEM_SLEEP_T);
}

void loop()
{
	if (wan_ok == 1) {
		push_temp_humi();
		if(realtime == 1)
			delay(mqttrate_sec*1000);
		else
			delay(httprate_min*60*1000);
	}
}
