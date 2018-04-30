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

static uint32_t wan_ok = 0;
#define	DATARATE_MIN		1

extern ctrl_status_t ctrl_st;

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

#ifdef CONFIG_ALEXA
void relay1_on_saved_and_pub()
{
	relay_status_t mst;
	os_memcpy(&mst, &(ctrl_st.relay_status), sizeof(relay_status_t));
	mst.r1 = 1;

	app_check_set_push_save(&mst);
}

void relay1_off_saved_and_pub()
{
	relay_status_t mst;
	os_memcpy(&mst, &(ctrl_st.relay_status), sizeof(relay_status_t));
	mst.r1 = 0;

	app_check_set_push_save(&mst);
}

void relay2_on_saved_and_pub()
{
	relay_status_t mst;
	os_memcpy(&mst, &(ctrl_st.relay_status), sizeof(relay_status_t));
	mst.r2 = 1;

	app_check_set_push_save(&mst);
}

void relay2_off_saved_and_pub()
{
	relay_status_t mst;
	os_memcpy(&mst, &(ctrl_st.relay_status), sizeof(relay_status_t));
	mst.r2 = 0;

	app_check_set_push_save(&mst);
}

void relay3_on_saved_and_pub()
{
	relay_status_t mst;
	os_memcpy(&mst, &(ctrl_st.relay_status), sizeof(relay_status_t));
	mst.r3 = 1;

	app_check_set_push_save(&mst);
}

void relay3_off_saved_and_pub()
{
	relay_status_t mst;
	os_memcpy(&mst, &(ctrl_st.relay_status), sizeof(relay_status_t));
	mst.r3 = 0;

	app_check_set_push_save(&mst);
}

void relay4_on_saved_and_pub()
{
	relay_status_t mst;
	os_memcpy(&mst, &(ctrl_st.relay_status), sizeof(relay_status_t));
	mst.r4 = 1;

	app_check_set_push_save(&mst);
}

void relay4_off_saved_and_pub()
{
	relay_status_t mst;
	os_memcpy(&mst, &(ctrl_st.relay_status), sizeof(relay_status_t));
	mst.r4 = 0;

	app_check_set_push_save(&mst);
}

void relay5_on_saved_and_pub()
{
	relay_status_t mst;
	os_memcpy(&mst, &(ctrl_st.relay_status), sizeof(relay_status_t));
	mst.r5 = 1;

	app_check_set_push_save(&mst);
}

void relay5_off_saved_and_pub()
{
	relay_status_t mst;
	os_memcpy(&mst, &(ctrl_st.relay_status), sizeof(relay_status_t));
	mst.r5 = 0;

	app_check_set_push_save(&mst);
}

upnp_dev_t upnp_devs[] = {
	{
		.esp_conn = NULL,
		.port = 80,
		.dev_voice_name = "relay 1",
		.model_name = "OpenPlug",
		.model_num = "1.0",
		.dev_type = WEMO_SWITCH,
		.way_on = relay1_on_saved_and_pub,
		.way_off = relay1_off_saved_and_pub
	},
	{
		.esp_conn = NULL,
		.port = 81,
		.dev_voice_name = "relay 2",
		.model_name = "OpenPlug",
		.model_num = "1.0",
		.dev_type = WEMO_SWITCH,
		.way_on = relay2_on_saved_and_pub,
		.way_off = relay2_off_saved_and_pub
	},
	{
		.esp_conn = NULL,
		.port = 82,
		.dev_voice_name = "relay 3",
		.model_name = "OpenPlug",
		.model_num = "1.0",
		.dev_type = WEMO_SWITCH,
		.way_on = relay3_on_saved_and_pub,
		.way_off = relay3_off_saved_and_pub
	},
	{
		.esp_conn = NULL,
		.port = 83,
		.dev_voice_name = "relay 4",
		.model_name = "OpenPlug",
		.model_num = "1.0",
		.dev_type = WEMO_SWITCH,
		.way_on = relay4_on_saved_and_pub,
		.way_off = relay4_off_saved_and_pub
	},
	{
		.esp_conn = NULL,
		.port = 84,
		.dev_voice_name = "relay 5",
		.model_name = "OpenPlug",
		.model_num = "1.0",
		.dev_type = WEMO_SWITCH,
		.way_on = relay5_on_saved_and_pub,
		.way_off = relay5_off_saved_and_pub
	}
};
#endif

/* {"m":"voice_name", "d":{"ch1":"channel 1", "ch2":"channel 2"}} */
void ICACHE_FLASH_ATTR push_voice_name()
{
	char *msg = (char *)os_zalloc(200);
	if (msg == NULL) {
		INFO("alloc voice name memory failed\r\n");
		return;
	}

	os_sprintf(msg, "{\"ch1\":\"%s\",\"ch2\":\"%s\",\"ch3\":\"%s\",\"ch4\":\"%s\",\"ch5\":\"%s\"}",
				ctrl_st.ch1_voice_name,
				ctrl_st.ch2_voice_name,
				ctrl_st.ch3_voice_name,
				ctrl_st.ch4_voice_name,
				ctrl_st.ch5_voice_name
			);

	mjyun_publish("set_voice_name", msg);
	INFO("Pushed voice name = %s\r\n", msg);

	os_free(msg);
	msg = NULL;
}

irom void push_cold_on()
{
	char msg[4];
	os_memset(msg, 0, 4);
	os_sprintf(msg, "%d", ctrl_st.cold_on);

	mjyun_publish("cold_on", msg);
}

irom void push_alexa_on()
{
	char msg[4];
	os_memset(msg, 0, 4);
	os_sprintf(msg, "%d", ctrl_st.alexa_on);

	mjyun_publish("alexa_on", msg);
}

irom void push_airkiss_nff_on()
{
	char msg[4];
	os_memset(msg, 0, 4);
	os_sprintf(msg, "%d", ctrl_st.airkiss_nff_on);

	mjyun_publish("airkiss_nff_on", msg);
}

static void mjyun_stated_cb(mjyun_state_t state)
{
    if (mjyun_state() != state)
        INFO("Platform: mjyun_state error \r\n");

    switch (state)
    {
        case WIFI_IDLE:
            INFO("Platform: WIFI_IDLE\r\n");
            break;
		case WIFI_SMARTLINK_START:
			INFO("Platform: WIFI_SMARTLINK_START\r\n");
			led_set_effect(0);
			break;
        case WIFI_SMARTLINK_LINKING:
            INFO("Platform: WIFI_SMARTLINK_LINKING\r\n");
			led_set_effect(1);
            break;
        case WIFI_SMARTLINK_FINDING:
            INFO("Platform: WIFI_SMARTLINK_FINDING\r\n");
			led_set_effect(0);
            break;
        case WIFI_SMARTLINK_TIMEOUT:
            INFO("Platform: WIFI_SMARTLINK_TIMEOUT\r\n");
            break;
        case WIFI_SMARTLINK_GETTING:
            INFO("Platform: WIFI_SMARTLINK_GETTING\r\n");
			led_set_effect(1);
            break;
        case WIFI_SMARTLINK_OK:
            INFO("Platform: WIFI_SMARTLINK_OK\r\n");
			led_set_effect(1);
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
#ifdef CONFIG_ALEXA
			int ret = 0;
			ret = upnp_start(upnp_devs, 5);
			if (ret != 0) {
				upnp_stop(upnp_devs, 5);
				ctrl_st.alexa_on = 0;
			}
#endif
			led_set_effect(1);
            break;
        case WIFI_STATION_ERROR:
            INFO("Platform: WIFI_STATION_ERROR\r\n");
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

void mjyun_receive(const char *event_name, const char *event_data)
{
	INFO("RECEIVED: key:value [%s]:[%s]", event_name, event_data);

	if (0 == os_strcmp(event_name, "set")) {
		cJSON * pD = cJSON_Parse(event_data);
		if ((NULL != pD) && (cJSON_Object == pD->type)) {
			cJSON * pR1 = cJSON_GetObjectItem(pD, "r1");
			cJSON * pR2 = cJSON_GetObjectItem(pD, "r2");
			cJSON * pR3 = cJSON_GetObjectItem(pD, "r3");
			cJSON * pR4 = cJSON_GetObjectItem(pD, "r4");
			cJSON * pR5 = cJSON_GetObjectItem(pD, "r5");

			relay_status_t mst;
			extern ctrl_status_t ctrl_st;
			mst.r1 = ctrl_st.relay_status.r1;
			mst.r2 = ctrl_st.relay_status.r2;
			mst.r3 = ctrl_st.relay_status.r3;
			mst.r4 = ctrl_st.relay_status.r4;
			mst.r5 = ctrl_st.relay_status.r5;

			if ((NULL != pR1) && (cJSON_Number == pR1->type)) {
				if (mst.r1 != pR1->valueint) {
					mst.r1 = pR1->valueint;
				}
			}
			if ((NULL != pR2) && (cJSON_Number == pR2->type)) {
				if (mst.r2 != pR2->valueint) {
					mst.r2 = pR2->valueint;
				}
			}
			if ((NULL != pR3) && (cJSON_Number == pR3->type)) {
				if (mst.r3 != pR3->valueint) {
					mst.r3 = pR3->valueint;
				}
			}
			if ((NULL != pR4) && (cJSON_Number == pR4->type)) {
				if (mst.r4 != pR4->valueint) {
					mst.r4 = pR4->valueint;
				}
			}
			if ((NULL != pR5) && (cJSON_Number == pR5->type)) {
				if (mst.r5 != pR5->valueint) {
					mst.r5 = pR5->valueint;
				}
			}

			app_check_set_push_save(&mst);
		} else {
			INFO("%s: Error when parse JSON\r\n", __func__);
		}

		cJSON_Delete(pD);
	}

	/* {"m":"set_cold_on", "d":1} */
	if (0 == os_strcmp(event_name, "set_cold_on")) {
		uint8_t cd_on = atoi(event_data);
		INFO("RX set cold_on %d Request!\r\n", cd_on);
		ctrl_st.cold_on = cd_on;
		param_save();
		push_cold_on();
	}
	/* {"m":"get_cold_on", "d":""} */
	if (0 == os_strcmp(event_name, "get_cold_on")) {
		INFO("RX Get cold_on Request!\r\n");
		push_cold_on();
	}

	/* {"m":"set_alexa_on", "d":1} */
	if (0 == os_strcmp(event_name, "set_alexa_on")) {
		uint8_t cd_on = atoi(event_data);
		INFO("RX set alexa_on %d Request!\r\n", cd_on);

		if (0 == cd_on) {

			upnp_ssdp_stop();

		} else if (1 == cd_on) {

			upnp_ssdp_stop();

			int ret = 0;
			ret = upnp_ssdp_start();
			if (ret != 0) {
				upnp_ssdp_stop();
				cd_on = 0;
			}
		}

		ctrl_st.alexa_on = cd_on;

		param_save();
		push_alexa_on();
	}
	/* {"m":"get_alexa_on", "d":""} */
	if (0 == os_strcmp(event_name, "get_alexa_on")) {
		INFO("RX Get alexa_on Request!\r\n");
		push_alexa_on();
	}

	/* {"m":"set_airkiss_nff_on", "d":1} */
	if (0 == os_strcmp(event_name, "set_airkiss_nff_on")) {
		uint8_t cd_on = atoi(event_data);
		INFO("RX set airkiss_nff_on %d Request!\r\n", cd_on);
		ctrl_st.airkiss_nff_on = cd_on;

		if (0 == cd_on) {
			mjyun_lan_stop();
		} else if (1 == cd_on) {
			mjyun_lan_stop();
			mjyun_lan_start();
		}

		param_save();
		push_airkiss_nff_on();
	}
	/* {"m":"get_airkiss_nff_on", "d":""} */
	if (0 == os_strcmp(event_name, "get_airkiss_nff_on")) {
		INFO("RX Get airkiss_nff_on Request!\r\n");
		push_airkiss_nff_on();
	}

	/* {"m":"set_voice_name", "d":{"ch1":"channel 1", "ch2":"channel 2"}} */
	if (0 == os_strcmp(event_name, "set_voice_name")) {
		INFO("RX set_voice_name = %s\r\n", event_data);

		cJSON * pD = cJSON_Parse(event_data);
		if ((NULL != pD) && (cJSON_Object == pD->type)) {
			cJSON * pR1 = cJSON_GetObjectItem(pD, "ch1");
			cJSON * pR2 = cJSON_GetObjectItem(pD, "ch2");
			cJSON * pR3 = cJSON_GetObjectItem(pD, "ch3");
			cJSON * pR4 = cJSON_GetObjectItem(pD, "ch4");
			cJSON * pR5 = cJSON_GetObjectItem(pD, "ch5");

			int len;
			bool need_update = false;
			char *pvn;
			if ((NULL != pR1) && (cJSON_String == pR1->type)) {

				pvn = pR1->valuestring;
				len = os_strlen(pvn);
				if ( len > 0 && len <= 31) {
			#ifdef CONFIG_ALEXA
					os_strcpy(upnp_devs[0].dev_voice_name, pvn);
			#endif
					need_update = true;
					os_strcpy(ctrl_st.ch1_voice_name, pvn);
				} else {
					INFO("RX Invalid ch1 voice name\r\n");
				}
			}
			if ((NULL != pR2) && (cJSON_String == pR2->type)) {

				pvn = pR2->valuestring;
				len = os_strlen(pvn);
				if ( len > 0 && len <= 31) {
			#ifdef CONFIG_ALEXA
					os_strcpy(upnp_devs[1].dev_voice_name, pvn);
			#endif
					need_update = true;
					os_strcpy(ctrl_st.ch2_voice_name, pvn);
				} else {
					INFO("RX Invalid ch2 voice name\r\n");
				}
			}

			if ((NULL != pR3) && (cJSON_String == pR3->type)) {

				pvn = pR3->valuestring;
				len = os_strlen(pvn);
				if ( len > 0 && len <= 31) {
			#ifdef CONFIG_ALEXA
					os_strcpy(upnp_devs[2].dev_voice_name, pvn);
			#endif
					need_update = true;
					os_strcpy(ctrl_st.ch3_voice_name, pvn);
				} else {
					INFO("RX Invalid ch3 voice name\r\n");
				}
			}

			if ((NULL != pR4) && (cJSON_String == pR4->type)) {

				pvn = pR4->valuestring;
				len = os_strlen(pvn);
				if ( len > 0 && len <= 31) {
			#ifdef CONFIG_ALEXA
					os_strcpy(upnp_devs[3].dev_voice_name, pvn);
			#endif
					need_update = true;
					os_strcpy(ctrl_st.ch4_voice_name, pvn);
				} else {
					INFO("RX Invalid ch4 voice name\r\n");
				}
			}

			if ((NULL != pR5) && (cJSON_String == pR5->type)) {

				pvn = pR5->valuestring;
				len = os_strlen(pvn);
				if ( len > 0 && len <= 31) {
			#ifdef CONFIG_ALEXA
					os_strcpy(upnp_devs[4].dev_voice_name, pvn);
			#endif
					need_update = true;
					os_strcpy(ctrl_st.ch5_voice_name, pvn);
				} else {
					INFO("RX Invalid ch5 voice name\r\n");
				}
			}

			if (need_update) {
			#ifdef CONFIG_ALEXA
				upnp_stop(upnp_devs, 5);
				upnp_start(upnp_devs, 5);
			#endif
				param_save();
			}
		}
	}

	/* {"m":"get_voice_name"} */
	/* {"m":"voice_name", "d":{"ch1":"channel 1", "ch2":"channel 2"}} */
	if (0 == os_strcmp(event_name, "get_voice_name")) {
		INFO("RX get_voice_name cmd\r\n");
		push_voice_name();
	}

	if(os_strncmp(event_data, "ota", 3) == 0)
	{
		INFO("OTA: upgrade the firmware!\r\n");
		mjyun_mini_ota_start("ota/dev/ctrl5relay/files");
	}
	if(os_strncmp(event_data, "reset", 5) == 0)
	{
		INFO("Reset the system!\r\n");
		system_restart();
	}
}

void mjyun_connected()
{
	// need to update the status into cloud
	app_push_status(NULL);
	push_voice_name();

	wan_ok = 1;

	// stop to show the wifi status
	wifi_led_disable();
}

void mjyun_disconnected()
{
	// show the wifi status
	wifi_led_enable();
}

const mjyun_config_t mjyun_conf = {
	"MJP7938261749",		/* MK Ctrl5relay */
	HW_VERSION,
	FW_VERSION,
	FW_VERSION,
	"Device Offline",
#ifdef LOW_POWER
	0,
#else
	WITH_MQTT,
#endif
};

void init_yun()
{
	mjyun_statechanged(mjyun_stated_cb);
	mjyun_ondata(mjyun_receive);
	mjyun_onconnected(mjyun_connected);
	mjyun_ondisconnected(mjyun_disconnected);

	mjyun_run(&mjyun_conf);
	wifi_set_sleep_type(MODEM_SLEEP_T);
}

#ifdef CONFIG_SENSOR
/* sensor supporting start */
irom void setup_tsl2561()
{
	if (!tsl2561_begin(TSL2561_ADDR_LOW)) {
		serial_print("Could not found TSL2561 sensor!\r\n");
	}

	/*
	 * You can change the gain on the fly, to adapt
	 * to brighter/dimmer light situations
	 */
	tsl2561_setGain(TSL2561_GAIN_0X);		// set no gain (for bright situtations)
	//tsl2561_setGain(TSL2561_GAIN_16X);		// set 16x gain (for dim situations)

	/*
	 * Changing the integration time gives you a longer
	 * time over which to sense light
	 *
	 * longer timelines are slower, but are good in very
	 * low light situtations!
	 *
	 */
	tsl2561_setTiming(TSL2561_INTEGRATIONTIME_13MS);	// shortest integration time (bright light)
}

irom void setup_bmp180()
{
	if (!bmp180_begin()) {
		serial_print("Could not find a valid BMP180 sensor!\r\n");
	}
}

irom void push_sensor_data()
{
	char t_buf[8];
	char h_buf[8];
	int pressure;
	int lumi;

	sht2x_reset();
	dtostrf(sht2x_GetTemperature(), 5, 1, t_buf),
	dtostrf(sht2x_GetHumidity(), 5, 1, h_buf);

	char *t = strstrip(t_buf);
	char *h = strstrip(h_buf);

	pressure = bmp180_readPressure() / 100;
	lumi = tsl2561_getLuminosity(TSL2561_VISIBLE);

	INFO("Temperature(C):\t\t\t%s\r\n", t);
	INFO("Humidity(%):\t\t\t%s\r\n", h);
	INFO("Pressure(hPa):\t\t\t%d\r\n", pressure);
	INFO("Luminosity:\t\t\t%d\r\n", lumi);
}
/* sensor end */
#endif

irom void setup()
{
#ifdef DEBUG
	uart_init(115200, 115200);
#endif
	os_delay_us(100);
	INFO("\r\n\r\n\r\n\r\n\r\n\r\n");
	INFO("\r\nWelcom to Noduino Open Ctrl5relay!\r\n");
	INFO("Current firmware is user%d.bin\r\n", system_upgrade_userbin_check()+1);
	INFO("%s", noduino_banner);

	param_init();
	relay_init();

	led_init();
	xkey_init();

	init_yun();

	wifi_set_sleep_type(MODEM_SLEEP_T);

#ifdef CONFIG_SENSOR
	wire_begin();
	setup_tsl2561();
	setup_bmp180();
#endif
}

void loop()
{
	if (wan_ok == 1) {
#ifdef CONFIG_SENSOR
		push_sensor_data();
		INFO("#####################################\r\n");
#endif
	}

	delay(DATARATE_MIN*60*1000);
}
