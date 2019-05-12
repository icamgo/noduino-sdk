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

struct hotbuf g_hb;

os_timer_t worker_timer;

static int network_state = 0;
static int wan_ok = 0;
static int read_push_flag = 0;

void worker();

irom char *dtostrf(double number, signed char width, unsigned char prec,
		   char *s)
{
	bool negative = false;

	if (isnan(number)) {
		strcpy(s, "nan");
		return s;
	}
	if (isinf(number)) {
		strcpy(s, "inf");
		return s;
	}

	char *out = s;

	int fillme = width;	// how many cells to fill for the integer part
	if (prec > 0) {
		fillme -= (prec + 1);
	}
	// Handle negative numbers
	if (number < 0.0) {
		negative = true;
		fillme--;
		number = -number;
	}
	// Round correctly so that print(1.999, 2) prints as "2.00"
	// I optimized out most of the divisions
	double rounding = 2.0;
	uint8_t i;
	for (i = 0; i < prec; ++i)
		rounding *= 10.0;
	rounding = 1.0 / rounding;

	number += rounding;

	// Figure out how big our number really is
	double tenpow = 1.0;
	int digitcount = 1;
	while (number >= 10.0 * tenpow) {
		tenpow *= 10.0;
		digitcount++;
	}

	number /= tenpow;
	fillme -= digitcount;

	// Pad unused cells with spaces
	while (fillme-- > 0) {
		*out++ = ' ';
	}

	// Handle negative sign
	if (negative)
		*out++ = '-';

	// Print the digits, and if necessary, the decimal point
	digitcount += prec;
	int8_t digit = 0;
	while (digitcount-- > 0) {
		digit = (int8_t) number;
		if (digit > 9)
			digit = 9;	// insurance
		*out++ = (char)('0' | digit);
		if ((digitcount == prec) && (prec > 0)) {
			*out++ = '.';
		}
		number -= digit;
		number *= 10.0;
	}

	// make sure the string is terminated
	*out = 0;
	return s;
}

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
#ifdef DEBUG
	INFO("RECEIVED: key:value [%s]:[%s]", event_name, event_data);
#endif

	if(os_strncmp(event_data, "ota", 3) == 0)
	{
#ifdef DEBUG
		INFO("OTA: upgrade the firmware!\r\n");
#endif
		mjyun_mini_ota_start("ota/dev/openplant/files");
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

void http_upload_error_handle()
{
	upload_fail_cnt++;
	if(upload_fail_cnt >= 3) {
		// failed about 5min
		INFO("http pushed failed %d times, reset the system\r\n", upload_fail_cnt);
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
#ifdef DEBUG
		INFO("%s: strlen(full_resp) = %d\r\n", __func__, strlen(full_resp));
		INFO("%s: resp = %s<EOF>\r\n", __func__, resp);
		INFO("%s: memory left=%d\r\n", __func__, system_get_free_heap_size());
#endif
		cJSON *root = cJSON_Parse(resp);
		if ((NULL != root) && (cJSON_Object == root->type)) {
			cJSON *msg = cJSON_GetObjectItem(root, "message");
			cJSON *mqtt = cJSON_GetObjectItem(root, "mqtt");

			if ((NULL != msg) && (cJSON_String == msg->type)
				&& (NULL != msg->valuestring)) {

				if (os_strncmp((char *)msg->valuestring, "OK", 2) != 0) {
					http_upload_error_handle();
				} else {
					INFO("http return message is not OK\r\n");
				}
			} else {
#ifdef DEBUG
				INFO("cjson message object error\r\n");
#endif
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
#ifdef DEBUG
				INFO("cjson mqtt object error\r\n");
#endif
			}

		} else {
#ifdef DEBUG
			INFO("cjson root object error\r\n");
#endif
		}
		cJSON_Delete(root);
	} else {
		http_upload_error_handle();
#ifdef DEBUG
		INFO("%s: http_status=%d\r\n", __func__, http_status);
#endif
	}
}

void http_upload(char *tt, char *hh, char *vbat, int light, int co2, uint32_t ts)
{
	uint8_t * URL = (uint8_t *) os_zalloc(os_strlen(HTTP_UPLOAD_URL) +
	                  os_strlen(mjyun_getdeviceid()) +
	                  os_strlen(tt) + os_strlen(hh) + os_strlen(vbat) +
	                  8 + 8 + 12 + 12);

	if ( URL == NULL ) {
#ifdef DEBUG
		INFO("%s: not enough memory\r\n", __func__);
#endif
		return;
	}

	uint8_t ma[6];
	char sta_mac[16];
	os_memset(sta_mac, 0, sizeof(sta_mac) / sizeof(char));
	wifi_get_macaddr(STATION_IF, ma);
	os_sprintf(sta_mac, "%02X%02X%02X%02X%02X%02X", ma[0], ma[1], ma[2], ma[3], ma[4], ma[5]);

	os_sprintf(URL,
	           HTTP_UPLOAD_URL,
	           mjyun_getdeviceid(),
	           (tt),
	           (hh),
			   (vbat),
			   light,
			   co2,
	           ts,
	           sta_mac);
	http_post((const char *)URL , "Content-Type:application/json\r\n", "", http_upload_cb);
#ifdef DEBUG
	INFO("%s\r\n", (char *)URL);
#endif
	os_free( URL );
}

irom void show_time()
{
	//struct tm *tblock;
	uint32_t cs = time(NULL);
	INFO("Current timestamp: %d\r\n", cs);
}

void mjyun_connected()
{
	show_time();

	os_timer_disarm(&worker_timer);
	os_timer_setfn(&worker_timer, (os_timer_func_t *) worker, NULL);
	os_timer_arm(&worker_timer, MQTT_RATE*1000, 1);

	mjyun_setssidprefix("NOD_");

	wan_ok = 1;

	// stop to show the wifi status
	wifi_led_disable();

	//wifi_set_sleep_type(LIGHT_SLEEP_T);
}

void mjyun_disconnected()
{
	// show the wifi status
	wifi_led_enable();
}

mjyun_config_t mjyun_conf = {
	"MKP2018102913",		/* Maike Noduino OpenPlant */
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
	} else {
		// low power mode
		mjyun_conf.run_flag |= WITHOUT_PING;
	}
	mjyun_run(&mjyun_conf);
}

static char g_temp[8];
static char g_humi[8];
static char g_vbat[8];
static int g_light = -1;
static int g_co2 = -1;

char *get_vbat(float *fv)
{
	int uv = mcp342x_get_uv();

	float vbat = uv * (99.985 + 219.35) / 99.985 / 1000000.0;

	dtostrf(vbat, 6, 2, g_vbat);

	if(fv != NULL)
		*fv = vbat;

	// Trim leading space
	unsigned char *p = g_vbat;
	while(isspace(*p)) p++;

	return p;
}

char *get_temp(float *ft)
{
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
	float humi = sht2x_GetHumidity();

	dtostrf(humi, 5, 1, g_humi);

	if(fh != NULL)
		*fh = humi;

	char *h = strstrip(g_humi);

	//INFO("Humidity(%RH): %s\r\n", h);

	return h;
}

void fetch_datapoint()
{
	struct datapoint *pdp = &(g_hb.datapoints[g_hb.cnt]);
	uint32_t ts = time(NULL);

	INFO("fetch datapoint @ time: %d...\r\n", ts);

	INFO("vbat = %s\r\n", get_vbat(&(pdp->vbat)));
	get_temp(&(pdp->temp));
	get_humi(&(pdp->humi)) ;

	pdp->light = -1;
	pdp->timestamp = ts;
}

void publish_sensor_data(char *tt, char *hh, char *vbat, int light, int co2)
{
	char msg[128];
	os_memset(msg, 0, 128);

	os_sprintf(msg, "{\"temp\":%s,\"humi\":%s,\"vbat\":%s,\"light\":%d,\"co2\":%d}",
				tt, hh, vbat, light, co2);

	mjyun_publishstatus(msg);
}

void push_datapoints()
{
	// for testing only now
	char *tt, *hh, *vv;

	uint8_t len = read_push_flag, i;
	INFO("push %d datapoint...\r\n", len);
	for(i = 0; i < len; i++) {
		dtostrf(g_hb.datapoints[i].vbat, 6, 2, g_vbat);
		vv = strstrip(g_vbat);

		dtostrf(g_hb.datapoints[i].temp, 5, 1, g_temp);
		tt = strstrip(g_temp);

		dtostrf(g_hb.datapoints[i].humi, 5, 1, g_humi);
		hh = strstrip(g_humi);

		http_upload(tt, hh, vv, g_light, g_co2, g_hb.datapoints[i].timestamp);
	}
}

// entry function when power on or wakeup from deep sleep
irom void user_init()
{

#ifdef DEBUG
	uart_init(115200, 115200);
	os_delay_us(1000000);
#endif
	INFO("\r\n\r\n\r\nCurrent firmware is user%d.bin\r\n", system_upgrade_userbin_check()+1);
	INFO("%s", noduino_banner);

	/* read the hot data from rtc memory */
	system_rtc_mem_read(RTC_MEM_START, (void *)&g_hb, sizeof(struct hotbuf));

	param_init();

	if (g_hb.bootflag != INIT_MAGIC || param_get_realtime() == 1) {

		INFO("\r\nCold boot up or Need push data or realtime mode. Flag: 0x%08X, cnt: %d\r\n", g_hb.bootflag, g_hb.cnt);

		mcp342x_init();
		mcp342x_set_oneshot();
		sht2x_init();

		// need to push the datapoints
		if (g_hb.cnt == MAX_DP_NUM - 1) {

			read_push_flag = g_hb.cnt;
		}

		/* mark the flag as warm boot in rtc mem */
		g_hb.bootflag = INIT_MAGIC;
		g_hb.cnt = 0;
		system_rtc_mem_write(RTC_MEM_START, (void *)&g_hb, sizeof(struct hotbuf));

		system_init_done_cb(init_yun);

	} else {
		INFO("\r\nWarm boot up, save the sensor data into rtc memory, cnt = %d\r\n", g_hb.cnt);

		/* maybe wakeup by key pressed */
		// show_oled();		// turn off 3s later

		/* range check and resets counter if needed */
		if(g_hb.cnt < 0 || g_hb.cnt >= MAX_DP_NUM)
			g_hb.cnt = 0;

		mcp342x_init();
		mcp342x_set_oneshot();
		sht2x_init();

		/* fetch sensor data, timestamp... */
		fetch_datapoint();

		g_hb.cnt++;

		/* Setup next sleep cycle */
		if (g_hb.cnt == MAX_DP_NUM - 1) {
			set_deepsleep_wakeup_normal();
			INFO("set deepsleep wakeup normal\r\n");

			// clear the flag to enter the cold boot up next time
			g_hb.bootflag = 0;

		} else {
			set_deepsleep_wakeup_no_rf();
			INFO("set deepsleep wakeup no rf\r\n");
		}

		system_rtc_mem_write(RTC_MEM_START, (void *)&g_hb, sizeof(struct hotbuf));

		/* enter deep sleep */
		INFO("Enter deep sleep...\r\n");
		cloud_disable_timer();
		system_deep_sleep(SLEEP_TIME);
	}
}

#ifdef CONFIG_CHECK_HOTDATA
static float pre_hot_data = 0;
#endif

static int32_t cnt = -1;

void worker()
{
	uint32_t test = 0, ts;

	char *tt, *hh, *vv;

	float hot_data = 0.0;

	INFO("Woker start working...\r\n");

	if (wan_ok == 1) {

		if(param_get_realtime() == 1) {

			ts = time(NULL);
			hh = get_humi(&hot_data);

#ifdef CONFIG_CHECK_HOTDATA
			if (fabsf(hot_data - pre_hot_data) > 1.0) {
				pre_hot_data = hot_data;
#endif
				tt = get_temp(NULL);
				vv = get_vbat(NULL);
				publish_sensor_data(tt, hh, vv, g_light, g_co2);

#ifdef CONFIG_CHECK_HOTDATA
				if (cnt != -1)
					http_upload(tt, hh, vv, g_light, g_co2, ts);
			}
#endif
		}

		if(param_get_realtime() != 1 && cnt >= 50) {
			/* enter deep sleep after cold boot up 50s later */
			system_rtc_mem_read(RTC_MEM_START, (void *)&test, 4);
			INFO("Enter deep sleep in woker... flag: 0x%08X\r\n", test);

			cloud_disable_timer();
			set_deepsleep_wakeup_no_rf();
			system_deep_sleep(SLEEP_TIME);
		}

		if (cnt * MQTT_RATE >= HTTP_RATE || cnt == -1) {

			// cold bootup first time or http_rate interval

			if (read_push_flag != 0) {
				/*
				 * We clear the warm boot flag last time and enter the cold bootup
				 * process this time when need to push the datapoints in rtc mem
				 *
				*/
				INFO("push the MAX_DP_NUM -1 datapoints\r\n");
				push_datapoints();

				read_push_flag = 0;
			}

			ts = time(NULL);

			tt = get_temp(NULL);
			hh = get_humi(NULL);
			vv = get_vbat(NULL);

			// need to push the sensor data via http
			http_upload(tt, hh, vv, g_light, g_co2, ts);

			cnt = 0;
		}

		cnt++;
	}
}
