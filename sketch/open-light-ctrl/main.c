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

extern system_status_t sys_status;
int need_turn = 0x55;

#ifdef CONFIG_ALEXA
irom void light_on_saved_and_pub()
{
	mcu_status_t mst;
	os_memcpy(&mst, &(sys_status.mcu_status), sizeof(mcu_status_t));
	mst.s = 1;

#ifdef CONFIG_GRADIENT
	if (sys_status.grad_on == 0) {
#endif
		set_light_status(&mst);
		app_check_mcu_save(&mst);
		app_push_status(&mst);
#ifdef CONFIG_GRADIENT
	} else {
		change_light_grad(&mst);
	}
#endif
}

irom void light_off_saved_and_pub()
{
	mcu_status_t mst;
	os_memcpy(&mst, &(sys_status.mcu_status), sizeof(mcu_status_t));
	mst.s = 0;

#ifdef CONFIG_GRADIENT
	if (sys_status.grad_on == 0) {
#endif
		set_light_status(&mst);
		app_check_mcu_save(&mst);
		app_push_status(&mst);
#ifdef CONFIG_GRADIENT
	} else {
		change_light_grad(&mst);
	}
#endif
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

irom void mjyun_stated_cb(mjyun_state_t state)
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

irom void mjyun_connected()
{
	/* need to update the status in cloud */
	app_push_status(NULL);

#ifdef CONFIG_ALEXA
	app_push_voice_name(upnp_devs[0].dev_voice_name);
#endif

	app_push_cold_on();
	app_push_grad_on();
	app_push_rgbw_bind();
	app_push_alexa_on();
	app_push_airkiss_nff_on();

	if (sys_status.airkiss_nff_on == 0) {
		mjyun_lan_stop();
	}
}

irom void mjyun_disconnected()
{
	/* show the wifi status */
}

irom int digital_read(uint8_t pin)
{
	// pin < 16
	return GPIP(pin);
}

irom void do_pir()
{
	uint32_t status = GPIE;		// same as GPIO_STATUS_ADDRESS
	GPIEC = status;		//clear them interrupts

	INFO("do_pir = 0x%X\r\n", status);

	if (0 == (status & 0x10)) {
		// interrupt of gpio4 is not enabled
		return;
	}

	ETS_GPIO_INTR_DISABLE();

	//gpio_st = GPIO_REG_READ(GPIO_STATUS_ADDRESS) <==> GPIE
	//GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_st) <==> GPIEC
	//INFO("st = 0x%X\r\n", gpio_st);

	//uint32_t savedPS = xt_rsil(15);	// stop other interrupts

	//bool pirl = GPIO_INPUT_GET(4);
	bool pirl = digital_read(4);

	if (pirl == true && sys_status.mcu_status.s == 0) {
		need_turn = ON;
		INFO("need on\r\n");
		//light_on_save_and_pub
	} else if (pirl == false && sys_status.mcu_status.s == 1) {
		need_turn = OFF;
		INFO("need off\r\n");
		//light_off_save_and_pub
	}

	//xt_wsr_ps(savedPS);

	ETS_GPIO_INTR_ENABLE();
}

irom void platform_init(void)
{
	gpio16_output_conf();
	gpio16_output_set(1);

	mjyun_statechanged(mjyun_stated_cb);

	//espnow_start();

	/* execute app_start_check() every one second */
	network_sys_timer_cb_reg(app_start_check);

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

	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U,FUNC_GPIO4);
	GPIO_DIS_OUTPUT(4);				/* disable output (change to input) */
	PIN_PULLUP_DIS(PERIPHS_IO_MUX_GPIO4_U);

	gpio_pin_intr_state_set(4, GPIO_PIN_INTR_ANYEDGE);

	ETS_GPIO_INTR_ATTACH(do_pir, NULL);
	ETS_GPIO_INTR_ENABLE();

	INFO("gpio4 = %d\r\n", digital_read(4));

	app_start_status();

	/* Init platform */
	platform_init();
}

irom void user_init()
{
	app_param_load();

	uart_init(BIT_RATE_115200, BIT_RATE_115200);

	mjpwm_cmd_t init_cmd = {
		.scatter = MJPWM_CMD_SCATTER_APDM,
		.frequency = MJPWM_CMD_FREQUENCY_DIVIDE_1,
		.bit_width = MJPWM_CMD_BIT_WIDTH_12,
		.reaction = MJPWM_CMD_REACTION_FAST,
		.one_shot = MJPWM_CMD_ONE_SHOT_DISABLE,
		.resv = 0,
	};

	mjpwm_init(PIN_DI, PIN_DCKI, 2, init_cmd);

	/* Light the led ASAP */
#ifdef CONFIG_GRADIENT
	if (sys_status.grad_on == 0) {
#endif
		if ((!is_warm_boot()) && (sys_status.cold_on != 0)) {
			sys_status.mcu_status.s = 1;
		}

		set_light_status(NULL);
#ifdef CONFIG_GRADIENT
	} else {
		mcu_status_t mt;
		os_memcpy(&mt, &(sys_status.mcu_status), sizeof(mcu_status_t));

		if (!is_warm_boot()) {
			/* boot up from cold state */
			if (sys_status.cold_on == 1) {
				/* don't care the saved state, turn on */
				sys_status.mcu_status.s = 0;
				mt.s = 1;
				change_light_grad(&mt);
			} else {
				/* case the saved off state */
				if (sys_status.mcu_status.s == 1) {
					sys_status.mcu_status.s = 0;
					mt.s = 1;
					change_light_grad(&mt);
				} else {
					INFO("Boot from cold state, saved state is off, do nothing\r\n");
				}
			}
		} else {
			/* warm boot up, just refresh the state */
			set_light_status(NULL);
		}
	}
#endif

	set_warm_boot_flag();
	system_init_done_cb(system_init_done);
}
