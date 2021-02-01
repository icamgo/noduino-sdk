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

system_status_t sys_status = {
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
	.sc_effect = NONE_EFFECT,
	.voice_name = ""
};

extern upnp_dev_t upnp_devs[];

static os_timer_t effect_timer;
static os_timer_t delay_timer;

static light_effect_t smart_effect = 0;
static app_state_t app_state = APP_STATE_NORMAL;

uint32_t rgb2hsl(mcu_status_t *c, hsl_t *h);
void hsl2rgb(hsl_t *h, mcu_status_t *rr);
void change_light_grad(mcu_status_t *to);

irom void app_push_status(mcu_status_t *st)
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

irom void app_push_voice_name(char *vname)
{
	/* {"m":"voice_name", "d":"room light"} */

	mjyun_publish("voice_name", vname);
	INFO("Pushed voice name = %s\r\n", vname);
}

irom void app_push_cold_on()
{
	char msg[4];
	os_memset(msg, 0, 4);
	os_sprintf(msg, "%d", sys_status.cold_on);

	mjyun_publish("cold_on", msg);
}

irom void app_push_alexa_on()
{
	char msg[4];
	os_memset(msg, 0, 4);
	os_sprintf(msg, "%d", sys_status.alexa_on);

	mjyun_publish("alexa_on", msg);
}

irom void app_push_airkiss_nff_on()
{
	char msg[4];
	os_memset(msg, 0, 4);
	os_sprintf(msg, "%d", sys_status.airkiss_nff_on);

	mjyun_publish("airkiss_nff_on", msg);
}

irom void app_push_grad_on()
{
	char msg[4];
	os_memset(msg, 0, 4);
	os_sprintf(msg, "%d", sys_status.grad_on);

	mjyun_publish("grad_on", msg);
}

irom void app_push_rgbw_bind()
{
	char msg[4];
	os_memset(msg, 0, 4);
	os_sprintf(msg, "%d", sys_status.rgbw_bind);

	mjyun_publish("rgbw_bind", msg);
}

irom void mjyun_receive(const char * event_name, const char * event_data)
{
	/* {"m":"set", "d":{"r":18,"g":100,"b":7,"w":0,"s":1}} */
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
		} else {
			INFO("%s: Error when parse JSON\r\n", __func__);
		}
		cJSON_Delete(pD);
	}

	/* {"m":"bri", "d":23} */
	if (0 == os_strcmp(event_name, "bri")) {
		int bri = atoi(event_data);
		INFO("RX set brightness %d Request!\r\n", bri);
		change_light_lum(bri);
	}

	/* {"m":"set_cold_on", "d":1} */
	if (0 == os_strcmp(event_name, "set_cold_on")) {
		uint8_t cd_on = atoi(event_data);
		INFO("RX set cold_on %d Request!\r\n", cd_on);
		sys_status.cold_on = cd_on;
		app_param_save();
		app_push_cold_on();
	}
	/* {"m":"get_cold_on", "d":""} */
	if (0 == os_strcmp(event_name, "get_cold_on")) {
		INFO("RX Get cold_on Request!\r\n");
		app_push_cold_on();
	}

	/* {"m":"set_grad_on", "d":1} */
	if (0 == os_strcmp(event_name, "set_grad_on")) {
		uint8_t go_on = atoi(event_data);
		INFO("RX set grad_on %d Request!\r\n", go_on);
		sys_status.grad_on = go_on;
		app_param_save();
		app_push_grad_on();
	}
	/* {"m":"get_grad_on", "d":""} */
	if (0 == os_strcmp(event_name, "get_grad_on")) {
		INFO("RX Get grad_on Request!\r\n");
		app_push_grad_on();
	}

	/* {"m":"set_rgbw_bind", "d":1} */
	if (0 == os_strcmp(event_name, "set_rgbw_bind")) {
		uint8_t rgbw_bind = atoi(event_data);
		INFO("RX set rgbw_bind %d Request!\r\n", rgbw_bind);
		sys_status.rgbw_bind = rgbw_bind;
		app_param_save();
		app_push_rgbw_bind();
	}
	/* {"m":"get_rgbw_bind", "d":""} */
	if (0 == os_strcmp(event_name, "get_rgbw_bind")) {
		INFO("RX Get rgbw_bind Request!\r\n");
		app_push_rgbw_bind();
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

		sys_status.alexa_on = cd_on;

		app_param_save();
		app_push_alexa_on();
	}
	/* {"m":"get_alexa_on", "d":""} */
	if (0 == os_strcmp(event_name, "get_alexa_on")) {
		INFO("RX Get alexa_on Request!\r\n");
		app_push_alexa_on();
	}

	/* {"m":"set_airkiss_nff_on", "d":1} */
	if (0 == os_strcmp(event_name, "set_airkiss_nff_on")) {
		uint8_t cd_on = atoi(event_data);
		INFO("RX set airkiss_nff_on %d Request!\r\n", cd_on);
		sys_status.airkiss_nff_on = cd_on;

		if (0 == cd_on) {
			mjyun_lan_stop();
		} else if (1 == cd_on) {
			mjyun_lan_stop();
			mjyun_lan_start();
		}

		app_param_save();
		app_push_airkiss_nff_on();
	}
	/* {"m":"get_airkiss_nff_on", "d":""} */
	if (0 == os_strcmp(event_name, "get_airkiss_nff_on")) {
		INFO("RX Get airkiss_nff_on Request!\r\n");
		app_push_airkiss_nff_on();
	}

	/* {"m":"set_voice_name", "d":"room light"} */
	if (0 == os_strcmp(event_name, "set_voice_name")) {
		INFO("RX set_voice_name = %s\r\n", event_data);
		int len = os_strlen(event_data);
		if ( len > 0 && len <= 31) {

#ifdef CONFIG_ALEXA
			os_strcpy(upnp_devs[0].dev_voice_name, event_data);
#endif
			// save to flash
			os_strcpy(sys_status.voice_name, event_data);
			app_param_save();
			app_push_voice_name(sys_status.voice_name);
		} else {
			INFO("RX Invalid voice name\r\n");
		}
	}
	/* {"m":"get_voice_name", "d":""} */
	if (0 == os_strcmp(event_name, "get_voice_name")) {
		INFO("RX get_voice_name cmd\r\n");
		app_push_voice_name(sys_status.voice_name);
	}

	/* {"m":"get_state", "d":""} */
	if (0 == os_strcmp(event_name, "get_state")) {
		INFO("RX Get status Request!\r\n");
		app_push_status(NULL);
	}

	if(os_strncmp(event_data, "ota", 3) == 0) {
		INFO("OTA: upgrade the firmware!\r\n");
		mjyun_mini_ota_start("ota/dev/openlight/files");
	}
}

irom uint32_t rgb2hsl(mcu_status_t *c, hsl_t *h)
{
	float r = (float)(c->r/255.0f);
	float g = (float)(c->g/255.0f);
	float b = (float)(c->b/255.0f);

	float max = fmax(fmax(r, g), b);
	float min = fmin(fmin(r, g), b);

	h->l = (max + min) / 2.0f;

	if (max == min) {
		h->h = h->s = 0.0f;
	} else {
		float d = max - min;
		h->s = (h->l > 0.5f) ? d / (2.0f - max - min) : d / (max + min);

		if (r > g && r > b)
			h->h = (g - b) / d + (g < b ? 6.0f : 0.0f);
		else if (g > b)
			h->h = (b - r) / d + 2.0f;
		else
			h->h = (r - g) / d + 4.0f;

		h->h /= 6.0f;
	}
}

irom float hue2rgb(float p, float q, float t)
{
	if (t < 0.0f)
		t += 1.0f;
	if (t > 1.0f)
		t -= 1.0f;
	if (t < 1.0f/6.0f)
		return p + (q - p) * 6.0f * t;
	if (t < 1.0f/2.0f)
		return q;
	if (t < 2.0f/3.0f)
		return p + (q - p) * (2.0f/3.0f - t) * 6.0f;

	return p;
}

irom void hsl2rgb(hsl_t *h, mcu_status_t *rr)
{
	float r, g, b;

	if (h->s == 0.0f) {
		r = g = b = h->l;
	} else {
		float q = h->l < 0.5f ? h->l * (1.0f + h->s) : h->l + h->s - h->l * h->s;
		float p = 2.0f * h->l - q;
		r = hue2rgb(p, q, h->h + 1.0f/3.0f);
		g = hue2rgb(p, q, h->h);
		b = hue2rgb(p, q, h->h - 1.0f/3.0f);
	}

	rr->r = (uint8_t)(r*255 + 0.5f);
	rr->g = (uint8_t)(g*255 + 0.5f);
	rr->b = (uint8_t)(b*255 + 0.5f);
}

#ifdef CONFIG_GRADIENT
irom void change_light_grad(mcu_status_t *to)
{
	//change hsl_cur to hsl_to
	mcu_status_t *st = &(sys_status.mcu_status);
	static mcu_status_t rgb;

	static float step = 0.0f;
	static float l_from = 2.2f;

	static hsl_t hsl_cur, hsl_to;
	static float l_to;

	if (l_from == 2.2f) {
		// first run

		rgb2hsl(st, &hsl_cur);
		rgb2hsl(to, &hsl_to);

		if (st->s == 0) {
			// current state is off
			// need to change from 0 to l_to firstly
			l_from = 0.0f;

		} else {
			l_from = hsl_cur.l;
		}

		if (to->s == 0) {
			l_to = 0.0f;
		} else {
			l_to = hsl_to.l;
		}

		if (l_from < l_to)
			step = 0.015;
		else
			step = -0.015;

		//step = (l_to - l_from) / 50.0f;
		// turn off need to notify others ASAP
		app_push_status(to);

		DEBUG("l_from = %d, step = %d, l_to = %d\r\n", (int)(l_from*1000), (int)(step*1000), (int)(l_to*1000));
	}

	if ((l_to != l_from) && (fabsf(l_to - l_from) >= fabsf(step))) {

		// y = x^1.2, y' = 1.2 * x .^ 1.2
		if (l_from == 0.0f)
			l_from += step;
		else
			l_from += (step * 1.5 * powf(l_from, 0.5));

		DEBUG("l_from = %d, l_to = %d\r\n", (int)(l_from*1000), (int)(l_to*1000));

		hsl_to.l = l_from;

		hsl2rgb(&hsl_to, &rgb);

		rgb.s = 1;
		set_light_status(&rgb);

		// update to global
		os_memcpy(st, &rgb, sizeof(mcu_status_t));

		DEBUG("rgbws: (%d, %d, %d, %d, %d)\r\n", rgb.r, rgb.g, rgb.b, rgb.w, rgb.s);

		os_timer_disarm(&effect_timer);
		os_timer_setfn(&effect_timer, (os_timer_func_t *)change_light_grad, (void *)to);
		os_timer_arm(&effect_timer, 15, 1);
	} else {
		INFO("light_gradient end\r\n");
		os_timer_disarm(&effect_timer);

		if (l_to == 0) {
			hsl_to.l = hsl_cur.l;
			rgb.s = 0;
		} else {
			rgb.s = 1;
			hsl_to.l = l_to;
		}

		hsl2rgb(&hsl_to, &rgb);

		set_light_status(&rgb);
		INFO("rgbws: (%d, %d, %d, %d, %d)\r\n", rgb.r, rgb.g, rgb.b, rgb.w, rgb.s);

		// update to global
		os_memcpy(st, &rgb, sizeof(mcu_status_t));
		app_param_save();

		app_push_status(&rgb);

		l_from = 2.2f;
	}
}
#endif

/*
 * bri = [0, 255]
 */
irom void change_light_lum(int bri)
{
	mcu_status_t mt;

	if (bri > 255)
		bri = 255;

	if (bri < 0)
		bri = 0;

	float l = bri / 255.0f;

	os_memcpy(&mt, &(sys_status.mcu_status), sizeof(mcu_status_t));

	hsl_t hsl;

	rgb2hsl(&mt, &hsl);

	hsl.l = l;

	hsl2rgb(&hsl, &mt);

//	if (bri == 0)
//		mt.s = 0;
//	else
		mt.s = 1;

#ifdef CONFIG_GRADIENT
	if (sys_status.grad_on == 0) {
#endif
		set_light_status(&mt);
		app_check_mcu_save(&mt);
		app_push_status(&mt);
#ifdef CONFIG_GRADIENT
	} else {
		change_light_grad(&mt);
	}
#endif
}

irom int get_light_lum()
{
	mcu_status_t *st = &(sys_status.mcu_status);
	hsl_t hsl;

	rgb2hsl(st, &hsl);

	return (int)(hsl.l * 255.0f + 0.5f);
}

irom bool get_light_on()
{
	mcu_status_t *st = &(sys_status.mcu_status);

	return st->s;
}

irom void set_light_status(mcu_status_t *st)
{
	if (st == NULL) {
		st = &(sys_status.mcu_status);
	}

	if ((st->r == st->g) && (st->g == st->b))
		st->w = st->r;
	else
		st->w = 0;

	if (st->s) {
		// we only change the led color when user setup apparently
		mjpwm_send_duty(
		    (uint16_t)(4095) * st->r / 255,
		    (uint16_t)(4095) * st->g / 255,
		    (uint16_t)(4095) * st->b / 255,
		    (uint16_t)(4095) * st->w / 255
		);
	} else {
		mjpwm_send_duty(0, 0, 0, 0);
	}
}

irom bool is_warm_boot()
{
	uint32_t warm_boot = 0;
	system_rtc_mem_read(64+20, (void *)&warm_boot, sizeof(warm_boot));
	//INFO("rtc warm_boot = %X\r\n", warm_boot);

	return (warm_boot == 0x66AA);
}

irom void set_warm_boot_flag()
{
	uint32_t warm_boot = 0x66AA;
	system_rtc_mem_write(64+20, (void *)&warm_boot, sizeof(warm_boot));
}

irom void app_param_load(void)
{
	system_param_load(
	    (APP_START_SEC),
	    0,
	    (void *)(&sys_status),
	    sizeof(sys_status));

	if (sys_status.init_flag == 0xff) {
		sys_status.init_flag = 1;
		os_strcpy(sys_status.voice_name, DEFAULT_VOICE_NAME);
	}

	int len = os_strlen(sys_status.voice_name);
	if (len == 0 || len >= 32) {
		// invalid voice name in flash
		os_strcpy(sys_status.voice_name, DEFAULT_VOICE_NAME);
		INFO("Invalid voice name in flash, reset to default name\r\n");
	}

	if (sys_status.cold_on == 0xff) {
		sys_status.cold_on = 1;
	}
	if (sys_status.alexa_on == 0xff) {
		sys_status.alexa_on = 1;
	}
	if (sys_status.airkiss_nff_on == 0xff) {
		sys_status.airkiss_nff_on = 1;
	}
	if (sys_status.grad_on == 0xff) {
		// reset the grad_on to 0x1 by default
		sys_status.grad_on = 1;
	}
	if (sys_status.rgbw_bind = 0xff) {
		sys_status.rgbw_bind = 0;
	}

	sys_status.start_count += 1;
	sys_status.start_continue += 1;
	app_param_save();

#ifdef CONFIG_ALEXA
	//copy the voice name to upnp_devs[]
	os_strcpy(upnp_devs[0].dev_voice_name, sys_status.voice_name);
#endif
}

irom void app_start_status()
{
	INFO("OpenLight APP: start count:%d, start continue:%d\r\n",
			sys_status.start_count, sys_status.start_continue);
}

irom void app_param_restore(void)
{
	spi_flash_erase_sector(APP_START_SEC);
}

irom void app_param_save(void)
{
	INFO("Flash Saved !\r\n");
	// sys_status.mcu_status = local_mcu_status;
	system_param_save_with_protect(
	    (APP_START_SEC),
	    (void *)(&sys_status),
	    sizeof(sys_status));
}

irom void app_check_mcu_save(mcu_status_t *st)
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

		app_param_save();
	}
}

irom void set_light_effect(light_effect_t effect)
{
	smart_effect = effect;
}

irom void light_effect_start()
{
	static uint16_t val = 0;
	static bool flag = true;

	switch (smart_effect) {
		case RED_GRADIENT:
			mjpwm_send_duty(val, 0, 0, 0);
			break;
		case GREEN_GRADIENT:
			mjpwm_send_duty(0, val, 0, 0);
			break;
		case BLUE_GRADIENT:
			mjpwm_send_duty(0, 0, val, 0);
			break;
		case WHITE_GRADIENT:
			mjpwm_send_duty(0, 0, 0, val);
			break;
		default:
			mjpwm_send_duty(val, val, val, val);
			break;
	}

	if (flag)
		val += 64;
	else
		val -= 64;
	if (val / 64 == 4095 / 64 - 1) {
		flag = false;
	}
	if (val / 64 == 0) {
		flag = true;
	}
	os_timer_disarm(&effect_timer);
	os_timer_setfn(&effect_timer, (os_timer_func_t *)light_effect_start, NULL);
	os_timer_arm(&effect_timer, 20, 1);
}

irom void light_effect_stop()
{
	os_timer_disarm(&effect_timer);
}

irom void factory_reset()
{
	mjyun_systemrecovery();
	system_restore();
	os_delay_us(2000);
	system_restart();
}

irom void app_start_check(uint32_t system_start_seconds)
{
	if ((sys_status.start_continue != 0) && (system_start_seconds > 5)) {
		sys_status.start_continue = 0;
		app_param_save();
	}

#if defined(APP_AGEING)
	if (sys_status.start_count >= 65535) {
		INFO("OpenLight APP: clean ageing\r\n");
		sys_status.start_count = 65534;
		app_param_save();
	} else if (sys_status.start_count <= 1) {
		INFO("OpenLight APP: begin ageing\r\n");

		mjpwm_send_duty(4095, 4095, 4095, 4095);
	}
#endif

	if (sys_status.start_continue >= 6) {
		if (APP_STATE_RESTORE != app_state) {
			INFO("OpenLight APP: system restore\r\n");
			app_state = APP_STATE_RESTORE;
			// Init flag and counter
			os_memset(&sys_status, 0, sizeof(system_status_t));
			sys_status.mcu_status.r = 75;
			sys_status.mcu_status.g = 255;
			sys_status.mcu_status.s = 1;
			sys_status.cold_on = 1;
			sys_status.grad_on = 1;
			sys_status.rgbw_bind = 0;
			sys_status.airkiss_nff_on = 1;
			os_strcpy(sys_status.voice_name, DEFAULT_VOICE_NAME);
			// Save param
			app_param_save();

			// waitting the mjyun_storage init is ok
			os_timer_disarm(&delay_timer);
			os_timer_setfn(&delay_timer, (os_timer_func_t *)factory_reset, NULL);
			os_timer_arm(&delay_timer, 2000, 0);
		}
	} else if (sys_status.start_continue >= 5) {

		light_effect_stop();
		mjpwm_send_duty(4095, 0, 0, 0);

	} else if (sys_status.start_continue >= 4) {

		light_effect_stop();
		mjpwm_send_duty(0, 4095, 0, 0);

	} else if (sys_status.start_continue >= 3) {
		if (APP_STATE_SMART != app_state) {
			INFO("OpenLight APP: User request to enter into smart config mode\r\n");
			app_state = APP_STATE_SMART;

			/* wait the network_init is ok */
			os_timer_disarm(&delay_timer);
			os_timer_setfn(&delay_timer, (os_timer_func_t *)mjyun_forceentersmartlinkmode, NULL);
			os_timer_arm(&delay_timer, 200, 0);

			light_effect_start();
		}
	}
	if ((WIFI_SMARTLINK_START == mjyun_state()) ||
	    (WIFI_SMARTLINK_LINKING == mjyun_state()) ||
	    (WIFI_SMARTLINK_FINDING == mjyun_state()) ||
	    (WIFI_SMARTLINK_GETTING == mjyun_state())) {
		if (APP_STATE_SMART != app_state) {
			INFO("OpenLight APP: User request to begin smart config effect\r\n");
			app_state = APP_STATE_SMART;

			light_effect_start();
		}
	} else if (APP_STATE_SMART == app_state &&
			(mjyun_state() == WIFI_SMARTLINK_OK ||
			 mjyun_state() == WIFI_AP_STATION_OK ||
			 mjyun_state() == WIFI_STATION_OK ||
			 mjyun_state() == MJYUN_CONNECTED)) {

		app_state = APP_STATE_NORMAL;

		set_light_status(NULL);
		app_push_status(NULL);

		light_effect_stop();
	}
}
