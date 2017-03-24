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

LOCAL struct keys_param keys;
LOCAL struct single_key_param *single_key[KEY_NUM];

iram void xkey_long_press(void)
{
	INFO("key long pressed\r\n");
	mjyun_systemrecovery();
	param_restore();
    system_restore();
    system_restart();
}

iram void xkey_short_press(void)
{
	// reverse the status of relay
	uint8_t st = (~relay_get_status()) & 0x1;
	INFO("key short pressed\r\n");

	param_set_status(st);
	param_save();

	relay_set_status_and_publish(st);
}

irom void xkey_init()
{
	single_key[0] = key_init_single (KEY_GPIO_NUM, KEY_GPIO_MUX, KEY_GPIO_FUNC,
							xkey_long_press, xkey_short_press);

	// key level is LOW when key is pressed
	single_key[0]->key_level = 0;

	keys.key_num = 1;
	keys.single_key = single_key;
	key_init(&keys);
}
