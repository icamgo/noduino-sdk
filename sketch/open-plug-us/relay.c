/*
 *  Copyright (c) 2017 - 2025 MaiKe Labs
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

void relay_init()
{
	pinMode(RELAY_GPIO_NUM, OUTPUT);
}

void relay_on()
{
	INFO("set relay on\n");
	led_on();
	digitalWrite(RELAY_GPIO_NUM, HIGH);
}

void relay_off()
{
	INFO("set relay off\n");
	led_off();
	digitalWrite(RELAY_GPIO_NUM, LOW);
}

void relay_set_status(uint8_t status)
{
	if(status == 0) {
		relay_off();
	} else if (status == 1) {
		relay_on();
	}
}

void relay_set_status_and_publish(uint8_t status)
{
	if(status == 0) {
		relay_off();
		mjyun_publishstatus("off");
	} else if (status == 1) {
		relay_on();
		mjyun_publishstatus("on");
	}
}

uint8_t relay_get_status()
{
	return digitalRead(RELAY_GPIO_NUM);
}

void relay_publish_status()
{
	uint8_t status = relay_get_status();
	if(status == 0) {
		mjyun_publishstatus("off");
	} else if (status == 1) {
		mjyun_publishstatus("on");
	}
}
