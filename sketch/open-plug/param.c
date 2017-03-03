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

extern upnp_dev_t upnp_devs[];

struct minik_saved_param minik_param;

void ICACHE_FLASH_ATTR param_set_status(uint8_t status)
{
	if (status != minik_param.status) {
		if (status > 1) {
			INFO("Error status input!\n");
			return;
		}

		minik_param.status = status;
	}
}

uint8_t ICACHE_FLASH_ATTR param_get_status(void)
{
	return minik_param.status;
}

void ICACHE_FLASH_ATTR param_save(void)
{
	spi_flash_erase_sector(PARAM_START_SEC + 0);
	spi_flash_write((PARAM_START_SEC + 0) * SPI_FLASH_SEC_SIZE,
			(uint32 *) & minik_param,
			sizeof(struct minik_saved_param));
}

void ICACHE_FLASH_ATTR param_init()
{
	spi_flash_read((PARAM_START_SEC + 0) * SPI_FLASH_SEC_SIZE,
		       (uint32 *) & minik_param,
		       sizeof(struct minik_saved_param));

	uint32_t warm_boot = 0;
	system_rtc_mem_read(64+20, (void *)&warm_boot, sizeof(warm_boot));
	//INFO("rtc warm_boot = %X\r\n", warm_boot);

	// init data of spi flash
	if (minik_param.status != 0xff) {
		if (warm_boot != 0x66AA) {
			if (minik_param.cold_on == 1) {
				INFO("Cold boot up, set the switch on!\r\n");
				minik_param.status = 1;
				param_save();
			}
			INFO("cold_on = 0x%x\r\n", minik_param.cold_on);

			warm_boot = 0x66AA;
			system_rtc_mem_write(64+20, (void *)&warm_boot, sizeof(warm_boot));
		} else {
			INFO("Warm boot up, use the status saved in flash!\r\n");
		}
	} else {
		INFO("Invalid status value, reset to 0!\n");
		minik_param.status = 0;
		os_strcpy(minik_param.voice_name, DEFAULT_VOICE_NAME);
		param_save();
	}

	int len = os_strlen(minik_param.voice_name);
	if (len == 0 || len >= 32) {
		// invalid voice name in flash
		INFO("Invalid voice name in flash, reset to default name\r\n");
		os_strcpy(minik_param.voice_name, DEFAULT_VOICE_NAME);
		param_save();
	}

#ifdef CONFIG_ALEXA
	//copy the voice name to upnp_devs[]
	os_strcpy(upnp_devs[0].dev_voice_name, minik_param.voice_name);
#endif

	//INFO("Saved Status is: %d\n", param_get_status());
}
