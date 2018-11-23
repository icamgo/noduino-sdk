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
#include "bh1750.h"
#include "compile.h"

static int mqtt_rate = 2; //2 second
static int http_rate = 60; //60 second

static int realtime = 0;
static int network_state = 0;
static int wan_ok = 0;

irom static void mjyun_stated_cb(mjyun_state_t state)
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
			network_state = WIFI_AP_STATION_OK;
            break;
        case WIFI_AP_STATION_ERROR:
            INFO("Platform: WIFI_AP_STATION_ERROR\r\n");
            break;
        case WIFI_STATION_OK:
            INFO("Platform: WIFI_STATION_OK\r\n");
			network_state = WIFI_STATION_OK;
            break;
        case WIFI_STATION_ERROR:
            INFO("Platform: WIFI_STATION_ERROR\r\n");
            break;
        case WIFI_STA_DISCONNECTED:
            INFO("Platform: WIFI_STA_DISCONNECTED\r\n");
			network_state = WIFI_STA_DISCONNECTED;
            break;
        case MJYUN_CONNECTING:
            INFO("Platform: MJYUN_CONNECTING\r\n");
            break;
        case MJYUN_CONNECTING_ERROR:
			wan_ok = 0;
            INFO("Platform: MJYUN_CONNECTING_ERROR\r\n");
            break;
        case MJYUN_CONNECTED:
			wan_ok = 1;
            INFO("Platform: MJYUN_CONNECTED \r\n");
            break;
        case MJYUN_DISCONNECTED:
			wan_ok = 0;
            INFO("Platform: MJYUN_DISCONNECTED\r\n");
            break;
        default:
            break;
	}
}

irom void mjyun_receive(const char *event_name, const char *event_data)
{
#ifdef CONFIG_DEBUG
	INFO("RECEIVED: key:value [%s]:[%s]", event_name, event_data);
#endif

	if(os_strncmp(event_data, "ota", 3) == 0)
	{
#ifdef CONFIG_DEBUG
		INFO("OTA: upgrade the firmware!\r\n");
#endif
		mjyun_mini_ota_start("ota/dev/openikair/files");
	}
}

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

void http_upload_cb(char *resp, int http_status, char *full_resp)
{
	if(HTTP_STATUS_GENERIC_ERROR != http_status) {
#ifdef CONFIG_DEBUG
		INFO("%s: strlen(full_resp)=%d\r\n", __func__, strlen(full_resp));
		INFO("%s: response=%s<EOF>\r\n", __func__, resp);
		INFO("%s: memory left=%d\r\n", __func__, system_get_free_heap_size());
#endif
		cJSON *root = cJSON_Parse(resp);
		if ((NULL != root) && (cJSON_Object == root->type)) {
			cJSON *msg = cJSON_GetObjectItem(root, "message");

			if ((NULL != msg) && (cJSON_String == msg->type)
				&& (NULL != msg->valuestring)) {

				if (os_strncmp((char *)msg->valuestring, "OK", 2) != 0) {
					http_error_handle();
				} else {
					INFO("http responsing message is not OK\r\n");
				}
			} else {
				INFO("cjson message object error\r\n");
			}
		} else {
			INFO("cjson root object error\r\n");
		}

	} else {
		http_error_handle();
#ifdef CONFIG_DEBUG
		INFO("%s: http_status=%d\r\n", __func__, http_status);
#endif
	}
}

void http_upload(char *tt, char *hh, int ll)
{
	uint8_t * URL = (uint8_t *) os_zalloc(os_strlen(HTTP_UPLOAD_URL) +
	                  os_strlen(mjyun_getdeviceid()) +
	                  os_strlen(tt) + os_strlen(hh) + 4 +
	                  32);

	if (URL == NULL) {
#ifdef CONFIG_DEBUG
		INFO("%s: not enough memory\r\n", __func__);
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
			   ll,
	           cs,
	           sta_mac);
	http_post((const char *)URL , "Content-Type:application/json\r\n", "", http_upload_cb);
#ifdef CONFIG_DEBUG
	INFO("%s\r\n", (char *)URL);
#endif
	os_free(URL);
}

#ifdef CONFIG_MQTT_ROBUST
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

irom void time_init()
{
	//struct tm *tblock;
	uint32_t cs = time(NULL);
	INFO("Current timestamp: %d\r\n", cs);
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
	HW_VERSION,
	FW_VERSION,
	FW_VERSION,
	"Device Offline",
	0,
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

static char g_temp[8];
static char g_humi[8];
static int g_light = -1;

char *get_temp(float *ft)
{
	sht2x_reset();
	float temp = sht2x_GetTemperature();

	dtostrf(temp, 5, 1, g_temp);

	if(ft != NULL)
		*ft = temp;

	char *t = strstrip(g_temp);
	//INFO("Temperature(C): %s\r\n", t);

	return t;
}

char *get_humi(float *fh)
{
	sht2x_reset();
	float humi = sht2x_GetHumidity();

	dtostrf(humi, 5, 1, g_humi);

	if(fh != NULL)
		*fh = humi;

	char *h = strstrip(g_humi);
	//INFO("Humidity(%RH): %s\r\n", h);

	return h;
}

int get_light()
{
	int lux = 0;

	/*
	 * The light sensor address is 0x5C
	 * The ADDR pin of BH1750 is pulled up (HIGH)
	*/
	bh1750_begin(0x5C);

	lux = bh1750_readLightLevel();

	return lux;
}

void publish_sensor_data(char *tt, char *hh, int ll)
{
	char msg[128];

	os_memset(msg, 0, 128);

	os_sprintf(msg, "{\"temp\":%s,\"humi\":%s,\"light\":%d}", tt, hh, ll);

	mjyun_publishstatus(msg);
}

irom void setup()
{
#ifdef CONFIG_DEBUG
	uart_init(115200, 115200);
#endif
	INFO("Current firmware is user%d.bin\r\n", system_upgrade_userbin_check()+1);
	INFO("%s", noduino_banner);

	realtime = 1;

	led_init();
	init_yun();

	wifi_set_sleep_type(MODEM_SLEEP_T);
}

#ifdef CONFIG_CHECK_HOTDATA
static float pre_hot_data = 0;
#endif

static uint32_t cnt = 0;

void loop()
{
	char *tt, *hh;
	int ll = -1;

#ifdef CONFIG_MQTT_ROBUST
	static uint32_t online_timer = 0;
#endif

	float hot_data = 0.0;

	if (wan_ok == 1) {

		cnt++;

		if(realtime == 1) {

			hh = get_humi(&hot_data);

#ifdef CONFIG_CHECK_HOTDATA
			if (fabsf(hot_data - pre_hot_data) > 2.0) {
				pre_hot_data = hot_data;
#endif
				tt = get_temp(NULL);
				ll = get_light();
				publish_sensor_data(tt, hh, ll);

#ifdef CONFIG_CHECK_HOTDATA
				http_upload(tt, hh, ll);

				if (cnt * mqtt_rate >= http_rate) {
					cnt = 0;
					goto next;
				}
			}
#endif
		}


		if (cnt * mqtt_rate >= http_rate) {
			tt = get_temp(NULL);
			hh = get_humi(NULL);
			ll = get_light();

			// need to push the sensor data via http
			http_upload(tt, hh, ll);

			cnt = 0;
		}
	}

#ifdef CONFIG_MQTT_ROBUST
	online_timer++;

	if(online_timer % (300/mqtt_rate) == 0) {
		// 5 min interval
		INFO("Checking the online state via http\r\n");
		check_online();
	}
#endif

next:
	delay(mqtt_rate*1000);
}
