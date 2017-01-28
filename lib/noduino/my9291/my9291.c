/*
 *  Copyright (c) 2016 - 2026 MaiKe Labs
 *
 *  Library for MY9291 LED driver Chip
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
#include "my9291.h"

static my9291_cmd_t my9291_commands;

static uint8_t pin_di = 13;
static uint8_t pin_dcki = 15;

irom void my9291_di_pulse(uint16_t times)
{
	uint16_t i;
	for (i = 0; i < times; i++) {
		digitalWrite(pin_di, HIGH);
		digitalWrite(pin_di, LOW);
	}
}

irom void my9291_dcki_pulse(uint16_t times)
{
	uint16_t i;
	for (i = 0; i < times; i++) {
		digitalWrite(pin_dcki, HIGH);
		digitalWrite(pin_dcki, LOW);
	}
}

irom void my9291_set_cmd(my9291_cmd_t command)
{
	uint8_t i;
	uint8_t command_data = *(uint8_t *) (&command);
	my9291_commands = command;

	// ets_intr_lock();
	// TStop > 12us.
	os_delay_us(12);
	// Send 12 DI pulse, after 6 pulse's falling edge store duty data, and 12
	// pulse's rising edge convert to command mode.
	my9291_di_pulse(12);
	// Delay >12us, begin send CMD data
	os_delay_us(12);
	// Send CMD data

	for (i = 0; i < 4; i++) {
		// DCK = 0;
		digitalWrite(pin_dcki, LOW);
		if (command_data & 0x80) {
			// DI = 1;
			digitalWrite(pin_di, HIGH);
		} else {
			// DI = 0;
			digitalWrite(pin_di, LOW);
		}
		// DCK = 1;
		digitalWrite(pin_dcki, HIGH);
		command_data = command_data << 1;
		if (command_data & 0x80) {
			// DI = 1;
			digitalWrite(pin_di, HIGH);
		} else {
			// DI = 0;
			digitalWrite(pin_di, LOW);
		}
		// DCK = 0;
		digitalWrite(pin_dcki, LOW);
		// DI = 0;
		digitalWrite(pin_di, LOW);
		command_data = command_data << 1;
	}

	// TStart > 12us. Delay 12 us.
	os_delay_us(12);
	// Send 16 DI pulse，at 14 pulse's falling edge store CMD data, and
	// at 16 pulse's falling edge convert to duty mode.
	my9291_di_pulse(16);
	// TStop > 12us.
	os_delay_us(12);
	// ets_intr_unlock();
}

irom void my9291_send_duty(uint16_t duty_r, uint16_t duty_g,
			   uint16_t duty_b, uint16_t duty_w)
{
	uint8_t i = 0;
	uint8_t channel = 0;
	uint8_t bit_length = 8;
	uint16_t duty_current = 0;

	// Definition for RGBW channels
	uint16_t duty[4] = { duty_r, duty_g, duty_b, duty_w };

	switch (my9291_commands.bit_width) {

		case MY9291_CMD_BIT_WIDTH_16:
			bit_length = 16;
			break;
		case MY9291_CMD_BIT_WIDTH_14:
			bit_length = 14;
			break;
		case MY9291_CMD_BIT_WIDTH_12:
			bit_length = 12;
			break;
		case MY9291_CMD_BIT_WIDTH_8:
			bit_length = 8;
			break;
		default:
			bit_length = 8;
			break;
	}

	// ets_intr_lock();
	// TStop > 12us.
	os_delay_us(12);

	for (channel = 0; channel < 4; channel++)	//RGBW 4CH
	{
		// RGBW Channel
		duty_current = duty[channel];
		// Send 8bit/12bit/14bit/16bit Data
		for (i = 0; i < bit_length / 2; i++) {
			// DCK = 0;
			digitalWrite(pin_dcki, LOW);
			if (duty_current & (0x01 << (bit_length - 1))) {
				// DI = 1;
				digitalWrite(pin_di, HIGH);
			} else {
				// DI = 0;
				digitalWrite(pin_di, LOW);
			}
			// DCK = 1;
			digitalWrite(pin_dcki, HIGH);
			duty_current = duty_current << 1;
			if (duty_current & (0x01 << (bit_length - 1))) {
				// DI = 1;
				digitalWrite(pin_di, HIGH);
			} else {
				// DI = 0;
				digitalWrite(pin_di, LOW);
			}
			//DCK = 0;
			digitalWrite(pin_dcki, LOW);
			//DI = 0;
			digitalWrite(pin_di, LOW);
			duty_current = duty_current << 1;
		}
	}

	// TStart > 12us. Ready for send DI pulse.
	os_delay_us(12);
	// Send 8 DI pulse. After 8 pulse falling edge, store old data.
	my9291_di_pulse(8);
	// TStop > 12us.
	os_delay_us(12);
	// ets_intr_unlock();
}

irom void my9291_init(uint8_t di, uint8_t dcki, my9291_cmd_t command)
{
	pin_di = di;
	pin_dcki = dcki;

	pinMode(pin_di, OUTPUT);
	pinMode(pin_dcki, OUTPUT);

	digitalWrite(pin_di, LOW);
	digitalWrite(pin_dcki, LOW);

	// Clear all duty register
	my9291_dcki_pulse(64 / 2);
	my9291_set_cmd(command);
}
