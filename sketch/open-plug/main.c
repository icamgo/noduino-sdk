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
#include "user_config.h"
#include "compile.h"

extern struct dev_param minik_param;

#ifdef CONFIG_ALEXA
void relay_on_saved_and_pub()
{
	relay_set_status_and_publish(1);
	param_set_status(1);
	param_save();
}

void relay_off_saved_and_pub()
{
	relay_set_status_and_publish(0);
	param_set_status(0);
	param_save();
}

bool get_relay_status()
{
	uint8_t rs = relay_get_status();

	if (minik_param.status != rs) {
		minik_param.status = rs;
	}

	return rs;
}

upnp_dev_t upnp_devs[] = {
	{
		.esp_conn = NULL,
		.port = 80,
		.dev_voice_name = DEFAULT_VOICE_NAME,
		.model_name = "OpenPlug",
		.model_num = "2.0",
		.dev_type = WEMO_SWITCH,
		.way_on = relay_on_saved_and_pub,
		.way_off = relay_off_saved_and_pub,
		.get_on = get_relay_status
	}
};
#endif

void ICACHE_FLASH_ATTR push_voice_name(char *vname)
{
	/* {"m":"voice_name", "d":"fan plug"} */

	mjyun_publish("voice_name", vname);
	INFO("Pushed voice name = %s\r\n", vname);
}

irom void push_cold_on()
{
	char msg[4];
	os_memset(msg, 0, 4);
	os_sprintf(msg, "%d", minik_param.cold_on);

	mjyun_publish("cold_on", msg);
}

irom void push_alexa_on()
{
	char msg[4];
	os_memset(msg, 0, 4);
	os_sprintf(msg, "%d", minik_param.alexa_on);

	mjyun_publish("alexa_on", msg);
}

irom void push_airkiss_nff_on()
{
	char msg[4];
	os_memset(msg, 0, 4);
	os_sprintf(msg, "%d", minik_param.airkiss_nff_on);

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
			ret = upnp_start(upnp_devs, 1);
			if (ret != 0) {
				upnp_stop(upnp_devs, 1);
				minik_param.alexa_on = 0;
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
	INFO("RECEIVED: key:value [%s]:[%s]", event_name, event_data);

	if(os_strncmp(event_data, "on", 2) == 0)
	{
		INFO("set switch on\r\n");
		param_set_status(1);
		param_save();
		relay_set_status_and_publish(1);
	}
	if(os_strncmp(event_data, "off", 3) == 0)
	{
		INFO("set switch off\r\n");
		param_set_status(0);
		param_save();
		relay_set_status_and_publish(0);
	}

	/* {"m":"set_cold_on", "d":1} */
	if (0 == os_strcmp(event_name, "set_cold_on")) {
		uint8_t cd_on = atoi(event_data);
		INFO("RX set cold_on %d Request!\r\n", cd_on);
		minik_param.cold_on = cd_on;
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

		minik_param.alexa_on = cd_on;

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
		minik_param.airkiss_nff_on = cd_on;

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

	/* {"m":"set_voice_name", "d":"fan plug"} */
	if (0 == os_strcmp(event_name, "set_voice_name")) {
		INFO("RX set_voice_name = %s\r\n", event_data);
		int len = os_strlen(event_data);
		if ( len > 0 && len <= 31) {

#ifdef CONFIG_ALEXA
			os_strcpy(upnp_devs[0].dev_voice_name, event_data);
#endif
			// save to flash
			os_strcpy(minik_param.voice_name, event_data);
			param_save();
			push_voice_name(minik_param.voice_name);
		} else {
			INFO("RX Invalid voice name\r\n");
		}
	}

	/* {"m":"get_voice_name"} */
	if (0 == os_strcmp(event_name, "get_voice_name")) {
		INFO("RX get_voice_name cmd\r\n");
		push_voice_name(minik_param.voice_name);
	}

	if(os_strncmp(event_data, "ota", 3) == 0)
	{
		INFO("OTA: upgrade the firmware!\r\n");
		mjyun_mini_ota_start("ota/dev/openplug/files");
	}
}

void mjyun_connected()
{
	// need to update the status in cloud
	relay_publish_status();
	push_voice_name(minik_param.voice_name);
	push_alexa_on();

	// stop to show the wifi status
	wifi_led_disable();
}

void mjyun_disconnected()
{
	// show the wifi status
	wifi_led_enable();
}

const mjyun_config_t mjyun_conf = {
	"MJP1004353832",		/* MK WiFi Plug, use WiFi Ctrl of WXMP */
	HW_VERSION,
	FW_VERSION,
	FW_VERSION,				/* 设备上线时，给app发送 online 消息中的附加数据，[选填] */
	"Device Offline",		/* 设备掉线时，给app发送 offline 消息中的附加数据，[选填] */
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

irom void setup()
{
#ifdef DEBUG
	uart_init(115200, 115200);
#endif
	os_delay_us(100);
	INFO("\r\n\r\n\r\n\r\n\r\n\r\n");
	INFO("\r\nWelcom to Noduino Open Plug!\r\n");
	INFO("Current firmware is user%d.bin\r\n", system_upgrade_userbin_check()+1);
	INFO("%s", noduino_banner);

	param_init();
	led_init();
	relay_init();
	xkey_init();

	// restore the relay status quickly
	relay_set_status(param_get_status());

	init_yun();
}

void loop()
{
	delay(10*1000);
}
