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
static int ready_push_flag = 0;

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
		mjyun_mini_ota_start("ota/dev/opentemp/files");
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

void http_upload(char *tt, char *vbat, uint32_t ts)
{
	uint8_t * URL = (uint8_t *) os_zalloc(os_strlen(HTTP_UPLOAD_URL) +
	                  os_strlen(mjyun_getdeviceid()) +
	                  os_strlen(tt) + os_strlen(vbat) +
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

	os_sprintf(URL,
	           HTTP_UPLOAD_URL,
	           mjyun_getdeviceid(),
	           (tt),
			   (vbat),
	           ts,
	           sta_mac);
	http_post((const char *)URL , "Content-Type:application/json\r\n", "", http_upload_cb);
	INFO("%s\r\n", (char *)URL);
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
	"MJP2024657532",		/* Maike OpenTemp */
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
static char g_vbat[8];

char *get_vbat(float *fv)
{
	int ad = system_adc_read();

	float delta = 82.0 + (ad - 639.0) * (22.0/172.0);

	float vbat = ((ad - delta) / 1024.0) * (200.0 + 1009.5) / 200.0;
	INFO("ad = %d, delta = %d\r\n", ad, (int) delta);

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
#ifdef USE_SHT2X
	float temp = sht2x_GetTemperature();
#else
	float temp = pt1000_get_temp();
#endif

	dtostrf(temp, 5, 1, g_temp);

	if(ft != NULL)
		*ft = temp;

	char *t = strstrip(g_temp);
	//INFO("Temperature(C): %s\r\n", t);

	return t;
}

void fetch_datapoint()
{
	struct datapoint *pdp = &(g_hb.datapoints[g_hb.cnt]);

	INFO("fetch datapoint...\r\n");

	INFO("vbat = %s\r\n", get_vbat(&(pdp->vbat)));
	get_temp(&(pdp->temp));

	g_hb.cnt++;		/* update the cnt */
}

void publish_sensor_data(char *tt, char *vbat)
{
	char msg[64];
	os_memset(msg, 0, 64);

	os_sprintf(msg, "{\"temp\":%s,\"vbat\":%s}", tt, vbat);

	mjyun_publishstatus(msg);
}

void push_datapoints()
{
	// for testing only now
	char *tt, *vv;
	uint32_t ts = 0;

	uint8_t ma[6];
	char mac[14];
	char j_dp[56];
	int body_len = 0;

	uint8_t len = ready_push_flag, i;
	INFO("push %d datapoint...\r\n", len);

	uint8_t *url = (uint8_t *) os_zalloc(os_strlen(DATA_PUSH_URL) +
	                  os_strlen(mjyun_getdeviceid()) + 14);
	if(url == NULL) {
		INFO("Request url mem faild...\r\n");
		return;
	}

	wifi_get_macaddr(STATION_IF, ma);
	os_sprintf(mac, "%02X%02X%02X%02X%02X%02X", ma[0], ma[1], ma[2], ma[3], ma[4], ma[5]);
	os_sprintf(url, DATA_PUSH_URL, mjyun_getdeviceid(), mac);

	INFO("url = %s, len = %d\r\n", url, os_strlen(url));

	// pack the data body
	// [{"t":-20.2,"v":3.84,"ts":1557711767},]

	char *body = (char *) os_zalloc(2+len*(3+6+5+5+5+5+5+10+5)+8);
	if(body == NULL) {
		INFO("Request http body mem faild...\r\n");
		return;
	}

	body[0] = '[';

	for(i = 0; i < len; i++) {
		dtostrf(g_hb.datapoints[i].vbat, 6, 2, g_vbat);
		vv = strstrip(g_vbat);

		dtostrf(g_hb.datapoints[i].temp, 5, 1, g_temp);
		tt = strstrip(g_temp);

		ts = g_hb.start_ts + i * SLEEP_TIME / 1000000;

		os_sprintf(j_dp, "{\"t\":%s,\"v\":%s,\"ts\":%d},", tt, vv, ts);
		INFO("http body item: %s, len = %d\r\n", j_dp, os_strlen(j_dp));

		os_strcat(body, j_dp);
	}

	body_len = os_strlen(body);
	body[body_len-1] = ']';
	http_post((const char *)url , "Content-Type:application/json\r\n", body, http_upload_cb);
	//INFO("%s\r\n", body);

	os_free(url);
	os_free(body);
}

void dev_pwr_on()
{
	//digitalWrite(15, LOW);
	digitalWrite(15, HIGH);
}

void dev_pwr_off()
{
	//digitalWrite(15, HIGH);
	digitalWrite(15, LOW);
}

// entry function when power on or wakeup from deep sleep
irom void user_init()
{

	char *tt, *vv;	

	pinMode(15, OUTPUT);	//pull down by default

	dev_pwr_on();

	os_delay_us(1000);		// mcp342x start-up time is 300us

#ifdef USE_MCP342X
	mcp342x_init();
#endif
#ifdef	USE_SHT2X
	sht2x_init();
#endif

	tt = get_temp(NULL);

	//TODO: show the temp on the OLED
	//oled_show(tt);

#ifdef DEBUG
	uart_init(115200, 115200);
	os_delay_us(1000000);
#endif
	INFO("\r\n\r\n\r\nCurrent firmware is user%d.bin\r\n", system_upgrade_userbin_check()+1);
	INFO("%s", noduino_banner);

	INFO("uv = %d\r\n", mcp342x_get_uv());
	INFO("Rt = %d\r\n", get_rt(mcp342x_get_uv()));
	INFO("Temperature: %s 'C\r\n", tt);

#ifdef LOW_POWER
	/* read the hot data from rtc memory */
	system_rtc_mem_read(RTC_MEM_START, (void *)&g_hb, sizeof(struct hotbuf));
#endif

	param_init();

#ifdef LOW_POWER
	if (g_hb.bootflag != INIT_MAGIC || param_get_realtime() == 1) {

		INFO("\r\nCold boot up or Need push data or realtime mode. Flag: 0x%08X, cnt: %d\r\n", g_hb.bootflag, g_hb.cnt);

		// need to push the datapoints
		if (g_hb.cnt == MAX_DP_NUM - 1) {

			fetch_datapoint();		/* fetch the datapoint this time */

			ready_push_flag = g_hb.cnt;
		}

		/* mark the flag as warm boot in rtc mem */
		g_hb.bootflag = INIT_MAGIC;
		g_hb.cnt = 0;
		system_rtc_mem_write(RTC_MEM_START, (void *)&g_hb, sizeof(struct hotbuf));
#endif
		system_init_done_cb(init_yun);

#ifdef LOW_POWER
	} else {
		INFO("\r\nWarm boot up, save the sensor data into rtc memory, cnt = %d\r\n", g_hb.cnt);

		/* range check and resets counter if needed */
		if(g_hb.cnt < 0 || g_hb.cnt >= MAX_DP_NUM) {
			INFO("cnt = %d is out of rang, reset it to 0\r\n", g_hb.cnt);
			g_hb.cnt = 0;
		}

		/* fetch sensor data, timestamp... */
		fetch_datapoint();

		dev_pwr_off();

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
		system_deep_sleep_instant(SLEEP_TIME);
	}
#endif
}

#ifdef CONFIG_CHECK_HOTDATA
static float pre_hot_data = 0;
#endif

static int32_t cnt = -1;

void worker()
{
	uint32_t test = 0, ts;

	char *tt, *vv;

	float hot_data = 0.0;

	INFO("Woker start working...\r\n");

	tt = get_temp(&hot_data);
	//TODO: show the temp on the OLED
	//oled_show(tt);

	if (wan_ok == 1) {

		if(param_get_realtime() == 1) {

			ts = time(NULL);

#ifdef CONFIG_CHECK_HOTDATA
			if (fabsf(hot_data - pre_hot_data) > 1.0) {
				pre_hot_data = hot_data;
#endif
				tt = get_temp(NULL);
				vv = get_vbat(NULL);
				publish_sensor_data(tt, vv);

#ifdef CONFIG_CHECK_HOTDATA
				if (cnt != -1)
					http_upload(tt, vv, ts);
			}
#endif
		}

		if (cnt == -1 || cnt * MQTT_RATE >= HTTP_RATE) {

			// cold bootup first time or http_rate interval

			if (ready_push_flag != 0) {
				/*
				 * We clear the warm boot flag last time and enter the cold bootup
				 * process this time when need to push the datapoints in rtc mem
				 *
				*/
				INFO("push the MAX_DP_NUM -1 datapoints\r\n");

				/* g_hb is filled from rtc mem when cold or warm bootup */

				push_datapoints();		/* push all the MAX_DP_NUM datapoints */

				ready_push_flag = 0;

			} else {

				ts = time(NULL);
				tt = get_temp(NULL);
				vv = get_vbat(NULL);
				// need to push the sensor data via http
				http_upload(tt, vv, ts);
			}

			cnt = 0;	// Reset the cnt
		}

	}// wan_ok

#ifdef LOW_POWER
	if(param_get_realtime() != 1 && cnt >= 3) {
		/* enter deep sleep after http post 3s */
		system_rtc_mem_read(RTC_MEM_START, (void *)&test, 4);
		INFO("Enter deep sleep in woker... flag: 0x%08X\r\n", test);

		cloud_disable_timer();
		set_deepsleep_wakeup_no_rf();

		/* init the start timestamp */
		g_hb.start_ts = time(NULL) + SLEEP_TIME/1000000;
		INFO("Set the start timestamp: %d\r\n", g_hb.start_ts);

		/* after waitting 50*mqtt_rate seconds, we need to fetch data this time */
		fetch_datapoint();

		system_rtc_mem_write(RTC_MEM_START, (void *)&g_hb, sizeof(struct hotbuf));

		system_deep_sleep_instant(SLEEP_TIME);
	}
#endif

	cnt++;

#ifdef USE_OLED
	if (cnt > 60) {

		dev_pwr_off();

		SET_PERI_REG_MASK(UART_CONF0(0), UART_TXFIFO_RST);//RESET FIFO
		CLEAR_PERI_REG_MASK(UART_CONF0(0), UART_TXFIFO_RST); 

		cloud_disable_timer();

		set_deepsleep_wakeup_normal();
		INFO("set deepsleep wakeup normal\r\n");

		system_deep_sleep_instant(0);
	}
#endif
}
