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

//uint8_t relay_pin = 4;	/* For v0.2 HEX-Relay board, D8(GPIO4) */
uint8_t relay_pin = 12;		/* D6(GPIO12) control the relay */

void relay_init()
{
	pinMode(relay_pin, OUTPUT);
}

void relay_on()
{
#ifdef DEBUG
	os_printf("set relay on\n");
#endif
	digitalWrite(relay_pin, HIGH);
}

void relay_off()
{
#ifdef DEBUG
	os_printf("set relay off\n");
#endif
	digitalWrite(relay_pin, LOW);
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
	return digitalRead(relay_pin);
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
