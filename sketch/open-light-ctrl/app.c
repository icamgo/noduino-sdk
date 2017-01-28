#include "user_config.h"

static system_status_t sys_status = {
	.init_flag = 0,
	.start_count = 0,
	.start_continue = 0,
	.mcu_status = {
		.s = 1,
		.r = 255,
		.g = 255,
		.b = 255,
		.w = 255,
	},
};

static os_timer_t app_smart_timer;
static os_timer_t delay_timer;
static app_state_t app_state = APP_STATE_NORMAL;
static uint8_t smart_effect = 0;

float ICACHE_FLASH_ATTR
fast_log2(float val)
{
	int * const  exp_ptr = (int *)(&val);
	int x = *exp_ptr;
	const int log_2 = ((x >> 23) & 255) - 128;
	x &= ~(255 << 23);
	x += 127 << 23;
	*exp_ptr = x;
	return (val + log_2);
}

void ICACHE_FLASH_ATTR app_push_status(mcu_status_t *st)
{
	char msg[48];
	os_memset(msg, 0, 48);

	if (st == NULL)
		st = &(sys_status.mcu_status);

	os_sprintf(msg, "{\"r\":%d,\"g\":%d,\"b\":%d,\"w\":%d,\"s\":%d}",
				st->r,
				st->g,
				st->b,
				st->w,
				st->s
			);

	mjyun_publishstatus(msg);
	INFO("Pushed status = %s\r\n", msg);
}

void ICACHE_FLASH_ATTR
mjyun_receive(const char * event_name, const char * event_data)
{
	/* {"m":"set", "d":{"r":18,"g":100,"b":7,"w":0,"s":1000}} */
	INFO("RECEIVED: key:value [%s]:[%s]\r\n", event_name, event_data);

	if (0 == os_strcmp(event_name, "set")) {
		cJSON * pD = cJSON_Parse(event_data);
		if ((NULL != pD) && (cJSON_Object == pD->type)) {
			cJSON * pR = cJSON_GetObjectItem(pD, "r");
			cJSON * pG = cJSON_GetObjectItem(pD, "g");
			cJSON * pB = cJSON_GetObjectItem(pD, "b");
			cJSON * pW = cJSON_GetObjectItem(pD, "w");
			cJSON * pS = cJSON_GetObjectItem(pD, "s");

			mcu_status_t mst;
			mst.r = sys_status.mcu_status.r;
			mst.g = sys_status.mcu_status.g;
			mst.b = sys_status.mcu_status.b;
			mst.w = sys_status.mcu_status.w;
			mst.s = sys_status.mcu_status.s;

			if ((NULL != pR) && (cJSON_Number == pR->type)) {
				if (mst.r != pR->valueint) {
					mst.r = pR->valueint;
				}
			}
			if ((NULL != pG) && (cJSON_Number == pG->type)) {
				if (mst.g != pG->valueint) {
					mst.g = pG->valueint;
				}
			}
			if ((NULL != pB) && (cJSON_Number == pB->type)) {
				if (mst.b != pB->valueint) {
					mst.b = pB->valueint;
				}
			}
			if ((NULL != pW) && (cJSON_Number == pW->type)) {
				if (mst.w != pW->valueint) {
					mst.w = pW->valueint;
				}
			}
			if ((NULL != pS) && (cJSON_Number == pS->type)) {
				if (mst.s != pS->valueint) {
					mst.s = pS->valueint;
				}
			}

			app_apply_settings(&mst);
			app_check_mcu_save(&mst);

			// notify the other users
			app_push_status(&mst);
		} else {
			INFO("%s: Error when parse JSON\r\n", __func__);
		}
		cJSON_Delete(pD);
	}

	if (0 == os_strcmp(event_name, "get")) {
		INFO("RX Get status Request!\r\n");
		app_push_status(NULL);
	}

	if(os_strncmp(event_data, "ota", 3) == 0) {
		INFO("OTA: upgrade the firmware!\r\n");
		mjyun_mini_ota_start("ota/dev/openlight/files");
	}
}

void ICACHE_FLASH_ATTR
app_apply_settings(mcu_status_t *st)
{
	if (st == NULL) {
		st = &(sys_status.mcu_status);
	}
	if (st->s) {
		// we only change the led color when user setup apparently
		mjpwm_send_duty(
		    PIN_DI,
		    PIN_DCKI,
		    (uint16_t)(4095) * st->r / 255,
		    (uint16_t)(4095) * st->g / 255,
		    (uint16_t)(4095) * st->b / 255,
		    (uint16_t)(4095) * st->w / 255
		);
	} else {
		mjpwm_send_duty(
		    PIN_DI,
		    PIN_DCKI,
		    0,
		    0,
		    0,
		    0
		);
	}
}

void ICACHE_FLASH_ATTR
app_load(void)
{
	system_param_load(
	    (APP_START_SEC),
	    0,
	    (void *)(&sys_status),
	    sizeof(sys_status));

	uint32_t warm_boot = 0;
	system_rtc_mem_read(64+20, (void *)&warm_boot, sizeof(warm_boot));
	//INFO("rtc warm_boot = %X\r\n", warm_boot);

	if (sys_status.init_flag) {
		if (warm_boot != 0x66AA) {
			INFO("Cold boot up, set the switch on!\r\n");
			sys_status.mcu_status.s = 1;

			warm_boot = 0x66AA;
			system_rtc_mem_write(64+20, (void *)&warm_boot, sizeof(warm_boot));
		} else {
			INFO("Warm boot up, use the status saved in flash!\r\n");
		}
	} else {
		sys_status.init_flag = 1;
	}
	sys_status.start_count += 1;
	sys_status.start_continue += 1;
	app_save();
}

void ICACHE_FLASH_ATTR app_start_status()
{
	INFO("Mjyun APP: start count:%d, start continue:%d\r\n",
			sys_status.start_count, sys_status.start_continue);
}

void ICACHE_FLASH_ATTR
app_save(void)
{
	INFO("Flash Saved !\r\n");
	// sys_status.mcu_status = local_mcu_status;
	system_param_save_with_protect(
	    (APP_START_SEC),
	    (void *)(&sys_status),
	    sizeof(sys_status));
}

void ICACHE_FLASH_ATTR
app_check_mcu_save(mcu_status_t *st)
{
	if(st == NULL)
		return;

	if(sys_status.mcu_status.r != st->r ||
			sys_status.mcu_status.g != st->g ||
			sys_status.mcu_status.b != st->b ||
			sys_status.mcu_status.w != st->w ||
			sys_status.mcu_status.s != st->s) {

		if(st->s == 1) {
			INFO("saved the new status into flash when mcu_status changed when led is on\r\n");
			sys_status.mcu_status.r = st->r;
			sys_status.mcu_status.g = st->g;
			sys_status.mcu_status.b = st->b;
			sys_status.mcu_status.w = st->w;
		}

		sys_status.mcu_status.s = st->s;

		app_save();
	}
}

void ICACHE_FLASH_ATTR
app_set_smart_effect(uint8_t effect)
{
	smart_effect = effect;
}

void ICACHE_FLASH_ATTR
app_smart_timer_tick()
{
	static uint16_t value = 0;

	static bool flag = true;

	// uint16_t rlog = 4095 - (uint16_t)(340 * fast_log2(4095 - value + 1));
	switch (smart_effect) {
	case 0:
		mjpwm_send_duty(
		    PIN_DI,
		    PIN_DCKI,
		    (uint16_t)value,
		    0,
		    0,
		    0
		);
		break;
	case 1:
		mjpwm_send_duty(
		    PIN_DI,
		    PIN_DCKI,
		    0,
		    (uint16_t)value,
		    0,
		    0
		);
		break;
	case 2:
		mjpwm_send_duty(
		    PIN_DI,
		    PIN_DCKI,
		    0,
		    0,
		    (uint16_t)value,
		    0
		);
		break;
	case 3:
		mjpwm_send_duty(
		    PIN_DI,
		    PIN_DCKI,
		    0,
		    0,
		    0,
		    (uint16_t)value
		);
		break;
	default:
		mjpwm_send_duty(
		    PIN_DI,
		    PIN_DCKI,
		    (uint16_t)value,
		    (uint16_t)value,
		    (uint16_t)value,
		    (uint16_t)value
		);
		break;
	}

	if (flag)
		value += 64;
	else
		value -= 64;
	if (value / 64 == 4095 / 64 - 1) {
		flag = false;
	}
	if (value / 64 == 0) {
		flag = true;
	}
	os_timer_disarm(&app_smart_timer);
	os_timer_setfn(&app_smart_timer, (os_timer_func_t *)app_smart_timer_tick, NULL);
	os_timer_arm(&app_smart_timer, 20, 1);
}

void ICACHE_FLASH_ATTR
factory_reset()
{
	mjyun_systemrecovery();
	system_restore();
	os_delay_us(2000);
	system_restart();
}

void ICACHE_FLASH_ATTR
app_start_check(uint32_t system_start_seconds)
{
	if ((sys_status.start_continue != 0) && (system_start_seconds > 5)) {
		sys_status.start_continue = 0;
		app_save();
	}

#if defined(APP_AGEING)
	if (sys_status.start_count >= 65535) {
		INFO("Mjyun APP: clean ageing\r\n");
		sys_status.start_count = 65534;
		app_save();
	} else if (sys_status.start_count <= 1) {
		INFO("Mjyun APP: begin ageing\r\n");
		mjpwm_send_duty(
		    PIN_DI,
		    PIN_DCKI,
		    4095,
		    4095,
		    4095,
		    4095
		);
	}
#endif

	if (sys_status.start_continue >= 6) {
		if (APP_STATE_RESTORE != app_state) {
			INFO("Mjyun APP: system restore\r\n");
			app_state = APP_STATE_RESTORE;
			// Init flag and counter
			sys_status.init_flag = 0;
			sys_status.start_continue = 0;
			// Save param
			app_save();

			// waitting the mjyun_storage init is ok
			os_timer_disarm(&delay_timer);
			os_timer_setfn(&delay_timer, (os_timer_func_t *)factory_reset, NULL);
			os_timer_arm(&delay_timer, 2000, 0);
		}
	} else if (sys_status.start_continue >= 5) {
		os_timer_disarm(&app_smart_timer);
		mjpwm_send_duty(
		    PIN_DI,
		    PIN_DCKI,
		    4095,
		    0,
		    0,
		    0
		);
	} else if (sys_status.start_continue >= 4) {
		os_timer_disarm(&app_smart_timer);
		mjpwm_send_duty(
		    PIN_DI,
		    PIN_DCKI,
		    0,
		    4095,
		    0,
		    0
		);
	} else if (sys_status.start_continue >= 3) {
		if (APP_STATE_SMART != app_state) {
			INFO("Mjyun APP: force into smart config mode\r\n");
			app_state = APP_STATE_SMART;

			/* wait the network_init is ok */
			os_timer_disarm(&delay_timer);
			os_timer_setfn(&delay_timer, (os_timer_func_t *)mjyun_forceentersmartlinkmode, NULL);
			os_timer_arm(&delay_timer, 200, 0);

			os_timer_disarm(&app_smart_timer);
			os_timer_setfn(&app_smart_timer, (os_timer_func_t *)app_smart_timer_tick, NULL);
			os_timer_arm(&app_smart_timer, 20, 1);
		}
	}
	if ((WIFI_SMARTLINK_START == mjyun_state()) ||
	    (WIFI_SMARTLINK_LINKING == mjyun_state()) ||
	    (WIFI_SMARTLINK_FINDING == mjyun_state()) ||
	    (WIFI_SMARTLINK_GETTING == mjyun_state())) {
		if (APP_STATE_SMART != app_state) {
			INFO("Mjyun APP: begin smart config effect\r\n");
			app_state = APP_STATE_SMART;
			os_timer_disarm(&app_smart_timer);
			os_timer_setfn(&app_smart_timer, (os_timer_func_t *)app_smart_timer_tick, NULL);
			os_timer_arm(&app_smart_timer, 20, 1);
		}
	} else if (APP_STATE_SMART == app_state &&
			(mjyun_state() == WIFI_SMARTLINK_OK ||
			 mjyun_state() == WIFI_AP_STATION_OK ||
			 mjyun_state() == WIFI_STATION_OK ||
			 mjyun_state() == MJYUN_CONNECTED)) {
		app_state = APP_STATE_NORMAL;
		app_apply_settings(NULL);
		app_push_status(NULL);
		os_timer_disarm(&app_smart_timer);
	}
}
