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

extern upnp_dev_t upnp_devs[];

ctrl_status_t ctrl_st;

irom void param_save(void)
{
	INFO("Parameters are saved to Flash !\r\n");
	spi_flash_erase_sector(PARAM_START_SEC + 0);
	spi_flash_write((PARAM_START_SEC + 0) * SPI_FLASH_SEC_SIZE,
					(uint32 *) & ctrl_st,
					sizeof(ctrl_status_t));
}

irom void param_load()
{
	spi_flash_read((PARAM_START_SEC + 0) * SPI_FLASH_SEC_SIZE,
		       (uint32 *) & ctrl_st,
		       sizeof(ctrl_status_t));

	// init data of spi flash
	if (ctrl_st.init_flag == 0xff) {
		INFO("Invalid parameters, reset to default!\n");
		os_memset(&ctrl_st, 0, sizeof(ctrl_status_t));

		ctrl_st.init_flag = 1; 

		os_strcpy(ctrl_st.ch1_voice_name, CH1_DEFAULT_VOICE_NAME);
		os_strcpy(ctrl_st.ch2_voice_name, CH2_DEFAULT_VOICE_NAME);
		os_strcpy(ctrl_st.ch3_voice_name, CH3_DEFAULT_VOICE_NAME);
		os_strcpy(ctrl_st.ch4_voice_name, CH4_DEFAULT_VOICE_NAME);
		os_strcpy(ctrl_st.ch5_voice_name, CH5_DEFAULT_VOICE_NAME);
	}

	int len = os_strlen(ctrl_st.ch1_voice_name);
	if (len == 0 || len >= 32) {
		// invalid voice name in flash
		os_strcpy(ctrl_st.ch1_voice_name, CH1_DEFAULT_VOICE_NAME);
		INFO("Invalid voice name in flash, reset to default name\r\n");
	}

	len = os_strlen(ctrl_st.ch2_voice_name);
	if (len == 0 || len >= 32) {
		// invalid voice name in flash
		os_strcpy(ctrl_st.ch2_voice_name, CH2_DEFAULT_VOICE_NAME);
		INFO("Invalid voice name in flash, reset to default name\r\n");
	}

	len = os_strlen(ctrl_st.ch3_voice_name);
	if (len == 0 || len >= 32) {
		// invalid voice name in flash
		os_strcpy(ctrl_st.ch3_voice_name, CH3_DEFAULT_VOICE_NAME);
		INFO("Invalid voice name in flash, reset to default name\r\n");
	}

	len = os_strlen(ctrl_st.ch4_voice_name);
	if (len == 0 || len >= 32) {
		// invalid voice name in flash
		os_strcpy(ctrl_st.ch4_voice_name, CH4_DEFAULT_VOICE_NAME);
		INFO("Invalid voice name in flash, reset to default name\r\n");
	}

	len = os_strlen(ctrl_st.ch5_voice_name);
	if (len == 0 || len >= 32) {
		// invalid voice name in flash
		os_strcpy(ctrl_st.ch5_voice_name, CH5_DEFAULT_VOICE_NAME);
		INFO("Invalid voice name in flash, reset to default name\r\n");
	}

#ifdef CONFIG_ALEXA
	//copy the voice name to upnp_devs[]
	os_strcpy(upnp_devs[0].dev_voice_name, ctrl_st.ch1_voice_name);
	os_strcpy(upnp_devs[1].dev_voice_name, ctrl_st.ch2_voice_name);
	os_strcpy(upnp_devs[2].dev_voice_name, ctrl_st.ch3_voice_name);
	os_strcpy(upnp_devs[3].dev_voice_name, ctrl_st.ch4_voice_name);
	os_strcpy(upnp_devs[4].dev_voice_name, ctrl_st.ch5_voice_name);
#endif

	ctrl_st.start_count++;
	param_save();
}

irom void app_push_status(relay_status_t *st)
{
	char msg[40];
	os_memset(msg, 0, 40);

	if (st == NULL)
		st = &(ctrl_st.relay_status);

	os_sprintf(msg, "{\"r1\":%d,\"r2\":%d,\"r3\":%d,\"r4\":%d,\"r5\":%d}",
				st->r1,
				st->r2,
				st->r3,
				st->r4,
				st->r5
			);

	mjyun_publishstatus(msg);
	INFO("Pushed status = %s\r\n", msg);
}

irom void app_check_set_push_save(relay_status_t *st)
{
	if(st == NULL)
		return;

	if(ctrl_st.relay_status.r1 != st->r1 ||
		ctrl_st.relay_status.r2 != st->r2 ||
		ctrl_st.relay_status.r3 != st->r3 ||
		ctrl_st.relay_status.r4 != st->r4 ||
		ctrl_st.relay_status.r5 != st->r5) {

		INFO("saved the new status into flash when relay_status changed\r\n");
		ctrl_st.relay_status.r1 = st->r1;
		ctrl_st.relay_status.r2 = st->r2;
		ctrl_st.relay_status.r3 = st->r3;
		ctrl_st.relay_status.r4 = st->r4;
		ctrl_st.relay_status.r5 = st->r5;

		relay_set_status(NULL);
		app_push_status(NULL);
		param_save();
	}
}

irom void toggle_status()
{
	ctrl_st.relay_status.r1 = (~ctrl_st.relay_status.r1) & 0x1;
	ctrl_st.relay_status.r2 = (~ctrl_st.relay_status.r2) & 0x1;
	ctrl_st.relay_status.r3 = (~ctrl_st.relay_status.r3) & 0x1;
	ctrl_st.relay_status.r4 = (~ctrl_st.relay_status.r4) & 0x1;
	ctrl_st.relay_status.r5 = (~ctrl_st.relay_status.r5) & 0x1;

	relay_set_status(NULL);
	app_push_status(NULL);
	param_save();
}
