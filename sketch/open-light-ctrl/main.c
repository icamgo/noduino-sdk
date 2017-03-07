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

#ifdef CONFIG_ALEXA
extern system_status_t sys_status;

void light_on_saved_and_pub()
{
	mcu_status_t mst;
	os_memcpy(&mst, &(sys_status.mcu_status), sizeof(mcu_status_t));
	mst.s = 1;

	if (sys_status.grad_on == 0) {
		set_light_status(&mst);
		app_check_mcu_save(&mst);
		app_push_status(&mst);
	} else {
		change_light_grad(&mst);
	}
}

void light_off_saved_and_pub()
{
	mcu_status_t mst;
	os_memcpy(&mst, &(sys_status.mcu_status), sizeof(mcu_status_t));
	mst.s = 0;

	if (sys_status.grad_on == 0) {
		set_light_status(&mst);
		app_check_mcu_save(&mst);
		app_push_status(&mst);
	} else {
		change_light_grad(&mst);
	}
}

upnp_dev_t upnp_devs[] = {
	{
		.esp_conn = NULL,
		.port = 80,
		.dev_voice_name = DEFAULT_VOICE_NAME,
		.model_name = "OpenLight Bulb",
		.model_num = "1.0",
		.dev_type = HUE,
		.way_on = light_on_saved_and_pub,
		.way_off = light_off_saved_and_pub,
		.set_pos = change_light_lum,
		.get_pos = get_light_lum,
		.get_on = get_light_on
	}
};
#endif

irom static void mjyun_stated_cb(mjyun_state_t state)
{
	if (mjyun_state() != state)
		INFO("Platform: mjyun_state error \r\n");

	switch (state) {
	case WIFI_IDLE:
		INFO("Platform: WIFI_IDLE\r\n");
		break;
	case WIFI_SMARTLINK_START:
		INFO("Platform: WIFI_SMARTLINK_START\r\n");
		set_light_effect(RED_GRADIENT);
		break;

	case WIFI_SMARTLINK_LINKING:
		INFO("Platform: WIFI_SMARTLINK_LINKING\r\n");
		set_light_effect(BLUE_GRADIENT);
		break;
	case WIFI_SMARTLINK_FINDING:
		INFO("Platform: WIFI_SMARTLINK_FINDING\r\n");
		set_light_effect(RED_GRADIENT);
		break;
	case WIFI_SMARTLINK_TIMEOUT:
		INFO("Platform: WIFI_SMARTLINK_TIMEOUT\r\n");
		break;
	case WIFI_SMARTLINK_GETTING:
		INFO("Platform: WIFI_SMARTLINK_GETTING\r\n");
		set_light_effect(GREEN_GRADIENT);
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
#ifdef CONFIG_ALEXA
		int ret = 0;
		ret = upnp_start(upnp_devs, 1);
		if (ret != 0) {
			upnp_stop(upnp_devs, 1);
			sys_status.alexa_on = 0;
		}
		if (sys_status.alexa_on == 0) {
			upnp_ssdp_stop();
		}
#endif
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

const mjyun_config_t mjyun_conf = {
	"MJP9040252280",
	HW_VERSION,
	FW_VERSION,
	FW_VERSION,
	"Device Offline",
	WITH_MQTT
};

void mjyun_connected()
{
	// need to update the status in cloud
	app_push_status(NULL);

#ifdef CONFIG_ALEXA
	app_push_voice_name(upnp_devs[0].dev_voice_name);
#endif

	app_push_cold_on();
	app_push_grad_on();
	app_push_alexa_on();
	app_push_airkiss_nff_on();

	if (sys_status.airkiss_nff_on == 0) {
		mjyun_lan_stop();
	}
}

void mjyun_disconnected()
{
	//show the wifi status
}

irom void platform_init(void)
{
	gpio16_output_conf();
	gpio16_output_set(1);

	mjyun_statechanged(mjyun_stated_cb);
	espnow_create();

	// execute app_start_check() every one second
	network_sys_timer_cb_reg(app_start_check);

	//app_start_check(0);

	mjyun_setssidprefix("NOD_");

	mjyun_ondata(mjyun_receive);
	mjyun_onconnected(mjyun_connected);
	mjyun_ondisconnected(mjyun_disconnected);
	mjyun_run(&mjyun_conf);
	wifi_set_sleep_type(MODEM_SLEEP_T);
}

irom void system_init_done()
{
	/* wait for uart is ok */
	os_delay_us(100);
	INFO("\r\n\r\n\r\n\r\n\r\n\r\n");
	INFO("\r\nWelcom to Noduino Open Light!\r\n");
	INFO("Current firmware is user%d.bin\r\n", system_upgrade_userbin_check()+1);
	INFO("%s", noduino_banner);

	app_start_status();

	// Init platform
	platform_init();
}

irom void user_init()
{
	app_param_load();

#define DEV_MODE 1
#if defined(DEV_MODE)
	uart_init(BIT_RATE_115200, BIT_RATE_115200);
#else
	// Set the port to print log info.
	UART_SetPrintPort(UART1);
	// Swap UART Port to RS485 bus
	system_uart_swap();
	// Set RS485 and LOG baudrate
	uart_init(BIT_RATE_9600, BIT_RATE_115200);
#endif

	mjpwm_cmd_t command = {
		.scatter = MJPWM_CMD_SCATTER_APDM,
		.frequency = MJPWM_CMD_FREQUENCY_DIVIDE_1,
		.bit_width = MJPWM_CMD_BIT_WIDTH_12,
		.reaction = MJPWM_CMD_REACTION_FAST,
		.one_shot = MJPWM_CMD_ONE_SHOT_DISABLE,
		.resv = 0,
	};

	mjpwm_init(PIN_DI, PIN_DCKI, 2, command);

	/* Light the led ASAP */
	set_light_status(NULL);

	system_init_done_cb(system_init_done);
}
