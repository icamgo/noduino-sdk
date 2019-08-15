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
#include "mcp342x.h"
#include "compile.h"

extern struct dev_param g_param;

static int mqtt_rate = 2; //2 second
static int http_rate = 600; //10 min

static int network_state = 0;
static int wan_ok = 0;

static char g_temp[8];
static char g_humi[8];
static char g_press[8];
static int g_light = -1;

static int g_ch4 = -1;	// in ppm

static int need_calibration = 0;
static int cali_delay_cnt = 0;

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
	float uv;

	INFO("RECEIVED: key:value [%s]:[%s]", event_name, event_data);

	if(os_strncmp(event_data, "on", 2) == 0)
	{
		INFO("set switch on\r\n");
		param_set_relay(1);
		param_save();
		relay_set_status_and_publish(1);
	}

	if(os_strncmp(event_data, "off", 3) == 0)
	{
		INFO("set switch off\r\n");
		param_set_relay(0);
		param_save();
		relay_set_status_and_publish(0);
	}

	if(os_strncmp(event_data, "ota", 3) == 0)
	{
		INFO("OTA: upgrade the firmware!\r\n");
		mjyun_mini_ota_start("ota/dev/opengas/files");
	}

	if(os_strncmp(event_data, "pubdata", 7) == 0)
	{
		INFO("publish the sensor data\r\n");
		g_ch4 = get_ch4(&uv);
		publish_sensor_data(g_ch4, (int)uv, (int)(g_param.v0 * 1000.0));
	}
	if(os_strncmp(event_data, "cali", 4) == 0)
	{
		INFO("clear the boot flag, restart to cold boot up...\r\n");
		uint32_t warm_boot = 0x0;
		system_rtc_mem_write(64+20, (void *)&warm_boot, sizeof(warm_boot));
		system_restart();
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

static uint32_t http_fail_cnt = 0;

void http_error_handle()
{
	http_fail_cnt++;
	if(http_fail_cnt >= 3) {
		// failed about 5min
		INFO("http pushed failed %d times, reset the system\r\n", http_fail_cnt);
		//system_restart();
	}
	//TODO: store the data in flash
}

/*
 * {"code": 0, "message": "ok", "mqtt": "enable"}
 *
*/
void http_upload_cb(char *resp, int http_status, char *full_resp)
{
	if (HTTP_STATUS_GENERIC_ERROR != http_status) {
		INFO("%s: strlen(full_resp) = %d\r\n", __func__, strlen(full_resp));
		INFO("%s: resp = %s<EOF>\r\n", __func__, resp);
		INFO("%s: memory left=%d\r\n", __func__, system_get_free_heap_size());

		cJSON *root = cJSON_Parse(resp);
		if ((NULL != root) && (cJSON_Object == root->type)) {
			cJSON *msg = cJSON_GetObjectItem(root, "message");
			cJSON *mqtt = cJSON_GetObjectItem(root, "mqtt");

			if ((NULL != msg) && (cJSON_String == msg->type)
				&& (NULL != msg->valuestring)) {

				if (os_strncmp((char *)msg->valuestring, "OK", 2) != 0) {
					http_error_handle();
				} else {
					INFO("http return message is not OK\r\n");
				}
			} else {
				INFO("cjson message object error\r\n");
			}

			if ((NULL != mqtt) && (cJSON_String == mqtt->type)
				&& (NULL != mqtt->valuestring)) {

				if (os_strncmp((char *)mqtt->valuestring, "enable", 6) == 0) {

					if (param_get_realtime() == 0) {

						param_set_realtime(1);
						param_save();

						INFO("Enable the MQTT, try to restart the system...\r\n");
						system_restart();
					}
				} else if (os_strncmp((char *)mqtt->valuestring, "disable", 7) == 0) {
					// check the mqtt flag
					if (param_get_realtime() != 0) {

						param_set_realtime(0);
						param_save();

						INFO("Disable the MQTT, try to restart the system...\r\n");
						system_restart();
					}

				}
			} else {
				INFO("cjson mqtt object error\r\n");
			}

		} else {
			INFO("cjson root object error\r\n");
		}
		cJSON_Delete(root);
	} else {
		http_error_handle();
		INFO("%s: http_status=%d\r\n", __func__, http_status);
	}
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

	float uv;

	if (need_calibration == 0) {
		g_ch4 = get_ch4(&uv);
		publish_sensor_data(g_ch4, (int)uv, (int)(g_param.v0 * 1000.0));
		http_upload(g_ch4);
	}

	// need to update the status in cloud
	relay_publish_status();

	// stop to show the wifi status
	wifi_led_disable();
}

void mjyun_disconnected()
{
	// show the wifi status
	wifi_led_enable();
}

mjyun_config_t mjyun_conf = {
	"MKP2018111619",		/* Maike Noduino OpenGas*/
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

	if (param_get_realtime() != 0) {
		INFO("MQTT is enabled in flash, run the cloud with mqtt \r\n");
		mjyun_conf.run_flag |= WITH_MQTT;
	}
	mjyun_run(&mjyun_conf);
}

int get_ch4(float *v)
{

	int uv, ppm;
	float delta = 0;

	uv = mcp342x_get_uv();

	if (v != NULL) {
		*v = (float) uv;
	}

	delta = uv/1000.0;

	delta -= g_param.v0;

	if (delta >= 0.0) {
		ppm = (int)(delta / 25.0 * 10000.0);
	} else {
		ppm = 0;
	}

	return ppm;
}

#ifdef CONFIG_EXTEND_SENSOR
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

char *get_press(float *fp)
{
	return NULL;
}

void publish_sensor_data(char *tt, char *hh, char *press, int light, int ch4)
{
	char msg[128];
	os_memset(msg, 0, 128);

	os_sprintf(msg, "{\"temp\":%s,\"humi\":%s,\"press\":%s,\"light\":%d,\"ch4\":%d}",
				tt, hh, press, light, ch4);

	mjyun_publishstatus(msg);
}

void http_upload(char *tt, char *hh, char *pp, int light, int ch4)
{
	uint8_t * URL = (uint8_t *) os_zalloc(os_strlen(HTTP_UPLOAD_URL) +
	                  os_strlen(mjyun_getdeviceid()) +
	                  os_strlen(tt) + os_strlen(hh) + os_strlen(pp) +
	                  8 + 8 + 12 + 12);

	if ( URL == NULL ) {
		INFO("%s: not enough memory\r\n", __func__);
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
			   (pp),
			   light,
			   ch4,
	           cs,
	           sta_mac);
	http_post((const char *)URL , "Content-Type:application/json\r\n", "", http_upload_cb);
	INFO("%s\r\n", (char *)URL);

	os_free( URL );
}
#else
void publish_sensor_data(int ch4, int uv, int v0)
{
	char msg[64];
	os_memset(msg, 0, 64);

	os_sprintf(msg, "{\"ch4\":%d,\"uv\":%d,\"v0\":%d}", ch4, uv, v0);

	mjyun_publishstatus(msg);
}

void http_upload(int ch4)
{
	uint8_t * URL = (uint8_t *) os_zalloc(os_strlen(HTTP_UPLOAD_URL) +
	                  os_strlen(mjyun_getdeviceid()) +
	                  8 + 8 + 12 + 12);

	if ( URL == NULL ) {
		INFO("%s: not enough memory\r\n", __func__);
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
			   ch4,
	           cs,
	           sta_mac);
	http_post((const char *)URL , "Content-Type:application/json\r\n", "", http_upload_cb);
	INFO("%s\r\n", (char *)URL);

	os_free( URL );
}
#endif

irom void setup()
{
#ifdef DEBUG
	uart_init(115200, 115200);
#endif
	INFO("\r\n\r\n\r\n\r\n\r\nNoduino OpenGas Current firmware is user%d.bin\r\n",
			system_upgrade_userbin_check()+1);
	INFO("%s", noduino_banner);

	uint32_t warm_boot = 0;
	system_rtc_mem_read(64+20, (void *)&warm_boot, sizeof(warm_boot));
	//INFO("rtc warm_boot = %X\r\n", warm_boot);

	if (warm_boot != 0x66AA) {

		INFO("Cold boot up!\r\n");
		// need to calibrate the gas sensor
		need_calibration = 1;
		cali_delay_cnt = 0;

		// save the warm boot flag into rtc mem
		warm_boot = 0x66AA;
		system_rtc_mem_write(64+20, (void *)&warm_boot, sizeof(warm_boot));
	}

	param_init();
	relay_init();

	mcp342x_init();
	mcp342x_set_oneshot();

	// restore the relay status quickly
	relay_set_status(param_get_relay());

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
	char *tt, *hh, *pp;

#ifdef CONFIG_MQTT_ROBUST
	static uint32_t online_timer = 0;
#endif
	float hot_data = 0.0;

	float uv = 0;

	if (wan_ok == 1 && need_calibration == 0) {

		cnt++;

		if(param_get_realtime() == 1) {

			g_ch4 = get_ch4(&uv);

			hot_data = g_ch4;

#ifdef CONFIG_CHECK_HOTDATA
			if (fabsf(hot_data - pre_hot_data) > 10.0) {
				// update when > 10ppm
				pre_hot_data = hot_data;
#endif
			#ifdef CONFIG_EXTEND_SENSOR
				tt = get_temp(NULL);
				hh = get_humi(NULL);
				pp = get_press(NULL);
				publish_sensor_data(tt, hh, pp, g_light, g_ch4);
			#else
				publish_sensor_data(g_ch4, (int)uv, (int)(g_param.v0 * 1000.0));
			#endif

#ifdef CONFIG_CHECK_HOTDATA
			#ifdef CONFIG_EXTEND_SENSOR
				http_upload(tt, hh, pp, g_light, g_ch4);
			#else
				http_upload(g_ch4);
			#endif

				if (cnt * mqtt_rate >= http_rate) {
					cnt = 0;
					goto next;
				}
			}
#endif
		}


		if (cnt * mqtt_rate >= http_rate) {

			g_ch4 = get_ch4(&uv);

#ifdef CONFIG_EXTEND_SENSOR
			tt = get_temp(NULL);
			hh = get_humi(NULL);
			pp = get_press(NULL);

			// need to push the sensor data via http
			http_upload(tt, hh, pp, g_light, g_ch4);
#else
			http_upload(g_ch4);
#endif
			cnt = 0;
		}
	}

	static float sum = 0;

	if (need_calibration == 1) {

		cali_delay_cnt++;

		if (cali_delay_cnt % 5 == 0) {
			// delay 5s, 10s, 15s...
			get_ch4(&uv);
			sum += uv;
		}

		if (cali_delay_cnt / 5 >= 18) {
			// calibration end, save the uv as mv
			g_param.v0 = sum / (cali_delay_cnt / 5) / 1000.0;

			INFO("Calibration end, V0 = %d mV\r\n", (int)(g_param.v0));

			need_calibration = 0;
			cali_delay_cnt = 0;

			g_ch4 = get_ch4(&uv);
			publish_sensor_data(g_ch4, (int)uv, (int)(g_param.v0 * 1000.0));

			sum = 0;
			uv = 0;

			param_save();
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
