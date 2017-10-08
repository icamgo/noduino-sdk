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
#include "noduino.h"
#include "spi.h"
#include "sx1278.h"

void sx1278_reset()
{
	digitalWrite(PWR, LOW);
	delay(200);
	digitalWrite(PWR, HIGH);
	delay(500);
}

void setup()
{
	serial_begin(115200);
	wifi_set_opmode(NULL_MODE);

	pinMode(4, OUTPUT);
	digitalWrite(4, HIGH);

	pinMode(PWR, OUTPUT);

	spi_init();

	sx1278_reset();

	sx1278_init();
}

void loop()
{
	serial_printf("Lora testing sketch\r\n");

	serial_printf("Lora opmode = 0x%x\r\n", sx1278_get_opmode());
	serial_printf("Lora rfmode = 0x%x\r\n", sx1278_get_rfmode());

	sx1278_write_reg(REG_LR_IRQFLAGS, 0xff);

	sx1278_send_data("lora.noduino.org", 16);
	serial_printf("Lora opmode = 0x%x\r\n", sx1278_get_opmode());

	delay(1000);

	sx1278_write_reg(REG_LR_IRQFLAGS, 0xff);

	delay(5000);
}
