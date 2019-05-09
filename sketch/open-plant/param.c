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

struct dev_param g_param;

irom uint8_t param_get_realtime()
{
	return g_param.realtime;
}

irom void param_set_realtime(uint8_t d)
{
	g_param.realtime = d;
}

irom void param_save(void)
{
	spi_flash_erase_sector(PARAM_START_SEC + 0);
	spi_flash_write((PARAM_START_SEC + 0) * SPI_FLASH_SEC_SIZE,
			(uint32 *) & g_param,
			sizeof(struct dev_param));
}

irom void param_erase_all(void)
{
	spi_flash_erase_sector(PARAM_START_SEC + 0);
}

irom void param_init()
{
	int need_to_save = 0;
	spi_flash_read((PARAM_START_SEC + 0) * SPI_FLASH_SEC_SIZE,
		       (uint32 *) & g_param,
		       sizeof(struct dev_param));

	if (g_param.realtime == 0xff) {
		INFO("Invalid param area, init the area...\n");
		g_param.temp = 0;
		g_param.humi = 0;
		g_param.light = 0;
		g_param.co2 = 0;

		g_param.realtime = 1;
		need_to_save = 1;
	}

	if (g_param.airkiss_nff_on == 0xff) {
		g_param.airkiss_nff_on = 1;
		need_to_save = 1;
	}

	if (need_to_save == 1) {
		param_save();
	}
}
