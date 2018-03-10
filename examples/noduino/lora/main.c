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
	serial_printf("Lora testing sketch\r\n");
}

void loop()
{
	serial_printf("Lora sync word = 0x%x\r\n", sx1278_get_syncword());
	serial_printf("Lora Preamble Len = %d\r\n", sx1278_get_preamble_len());
	//serial_printf("Lora RSSI = %d\r\n", sx1278_get_rssi());

	uint8_t f[3];
	sx1278_get_rf_freq(f);
	serial_printf("Lora Freq = 0x%X%02X%02X\r\n", f[0], f[1], f[2]);
	serial_printf("Lora Spread Factor = %d\r\n", sx1278_get_spread_fact());

	serial_printf("Lora modem conf1 = 0x%x\r\n", sx1278_read_reg(REG_MODEMCONFIG1));
	serial_printf("Lora modem conf2 = 0x%x\r\n", sx1278_read_reg(REG_MODEMCONFIG2));
	serial_printf("Lora modem conf3 = 0x%x\r\n", sx1278_read_reg(REG_MODEMCONFIG3));

	serial_printf("Lora opmode = 0x%x\r\n", sx1278_get_opmode());
	serial_printf("Lora rfmode = 0x%x\r\n", sx1278_get_rfmode());

	sx1278_write_reg(REG_IRQFLAGS, 0xff);

	sx1278_send_data("noduino.org", 11);
	serial_printf("Lora opmode = 0x%x\r\n\r\n", sx1278_get_opmode());

	delay(1000);

	sx1278_write_reg(REG_IRQFLAGS, 0xff);

	wifi_set_sleep_type(MODEM_SLEEP_T);
	delay(4000);
}
