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

void http_upload_cb(char *response, int http_status, char *full_response)
{
	if(HTTP_STATUS_GENERIC_ERROR != http_status) {
#ifdef DEBUG
		INFO( "%s: strlen(full_response)=%d\r\n", __func__, strlen( full_response ) );
		INFO( "%s: response=%s<EOF>\r\n", __func__, response );
		INFO( "%s: memory left=%d\r\n", __func__, system_get_free_heap_size() );
#endif

		if(os_strncmp(response, "ok", 2) != 0)
			http_upload_error_handle();

	} else {
		http_upload_error_handle();
#ifdef DEBUG
		INFO( "%s: http_status=%d\r\n", __func__, http_status );
#endif
	}
}

void http_upload(char *tt, char *hh, char *vbat, int light, int co2)
{
	uint8_t * URL = (uint8_t *) os_zalloc(os_strlen(HTTP_UPLOAD_URL) +
	                  os_strlen(mjyun_getdeviceid()) +
	                  os_strlen(tt) + os_strlen(hh) + os_strlen(vbat) +
	                  8 + 8 + 12 + 12);

	if ( URL == NULL ) {
#ifdef DEBUG
		INFO( "%s: not enough memory\r\n", __func__ );
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
			   (vbat),
			   light,
			   co2,
	           cs,
	           sta_mac);
	http_post((const char *)URL , "Content-Type:application/json\r\n", "", http_upload_cb);
#ifdef DEBUG
	INFO("%s\r\n", (char *)URL);
#endif
	os_free( URL );
}

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

	if (realtime == 1)
		mjyun_conf.run_flag |= WITH_MQTT;
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

void publish_sensor_data(char *tt, char *hh, char *vbat, int light, int co2)
{
	char msg[128];
	os_memset(msg, 0, 128);

	os_sprintf(msg, "{\"temp\":%s,\"humi\":%s,\"vbat\":%s,\"light\":%d,\"co2\":%d}",
				tt, hh, vbat, light, co2);

	mjyun_publishstatus(msg);
}

irom void setup()
{
#ifdef DEBUG
	uart_init(115200, 115200);
#endif
	INFO("Current firmware is user%d.bin\r\n", system_upgrade_userbin_check()+1);
	INFO("%s", noduino_banner);

	realtime = 1;

	mcp342x_init();
	mcp342x_set_oneshot();

	led_init();
	init_yun();

	wifi_set_sleep_type(MODEM_SLEEP_T);
}

//static float pre_hot_data = 0;

static uint32_t cnt = 0;

void loop()
{
	char *tt, *hh, *vv;

	if (wan_ok == 1) {

		cnt++;

		if(realtime == 1) {
			tt = get_temp(NULL);
			hh = get_humi(NULL);
			vv = get_vbat(NULL);
			publish_sensor_data(tt, hh, vv, g_light, g_co2);
		}


		if (cnt * 2 >= http_rate) {
			tt = get_temp(NULL);
			hh = get_humi(NULL);
			vv = get_vbat(NULL);

			// need to push the sensor data via http
			http_upload(tt, hh, vv, g_light, g_co2);

			cnt = 0;
		}
	}

	delay(mqtt_rate*1000);
}
