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

// lora tx node
int tx = 1;
uint8_t recv[255];

void sleep_avr()
{
	// sleep mode is set here
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
	// System sleeps here
	sleep_mode();
}

void sx1278_reset()
{
	digital_write(PWR, LOW);
	_delay_ms(200);
	digital_write(PWR, HIGH);
	_delay_ms(500);
}

int main(void)
{
	uint8_t irq_flag = 0;
	uint8_t crc = 0;
	pin_mode(4, OUTPUT);
	digital_write(4, HIGH);

	pin_mode(PWR, OUTPUT);

	spi_init();

	sx1278_reset();

	sx1278_init();

	//sleep_avr();
	while (1) {

		if (tx == 1) {

			sx1278_write_reg(REG_LR_IRQFLAGS, 0xff);

			sx1278_send_data("lora.noduino.org", 16);

			_delay_ms(1000);

			sx1278_write_reg(REG_LR_IRQFLAGS, 0xff);

			_delay_ms(5000);
		} else {
#if 0
			irq_flag = sx1278_read_reg(REG_LR_IRQFLAGS);

			if (irq_flag > 0) {
				if ((irq_flag & 0x40) == 0x40) {

					crc = sx1278_read_reg(REG_LR_MODEMCONFIG2);
					if (crc & 0x04 == 0x04) {
						sx1278_write_reg(REG_LR_FIFOADDRPTR, 0x00);
						int len = sx1278_read_reg(REG_LR_NBRXBYTES);

						spi_read_buf(0x0, recv, len);
						recv[len] = '\0';

						if(strncmp(recv,"lora.noduino.org",16)){
							_delay_ms(100);
						}
					}

					sx1278_set_opmode(STANDBY);
					sx1278_write_reg(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);
					sx1278_write_reg(REG_LR_HOPPERIOD, PACKET_MIAX_Value);
					sx1278_write_reg(REG_LR_DIOMAPPING1, 0X00);
					sx1278_write_reg(REG_LR_DIOMAPPING2, 0x00);
					sx1278_set_opmode(RX);

				} else if ((irq_flag & 0x08) == 0x08) {

					sx1278_set_opmode(STANDBY);
					sx1278_write_reg(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);
					sx1278_write_reg(REG_LR_HOPPERIOD, PACKET_MIAX_Value);
					sx1278_write_reg(REG_LR_DIOMAPPING1, 0X00);
					sx1278_write_reg(REG_LR_DIOMAPPING2, 0x00);
					sx1278_set_opmode(RX);

				} else if ((irq_flag & 0x04) == 0x04) {
					if ((irq_flag & 0x01) == 0x01) {
						sx1278_set_opmode(STANDBY);
						sx1278_write_reg(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);
						sx1278_write_reg(REG_LR_HOPPERIOD, PACKET_MIAX_Value);
						sx1278_write_reg(REG_LR_DIOMAPPING1, 0X02);
						sx1278_write_reg(REG_LR_DIOMAPPING2, 0x00);
						sx1278_set_opmode(RX);
					} else {                          
						sx1278_set_opmode(STANDBY);
						sx1278_write_reg(REG_LR_IRQFLAGSMASK,
								IRQN_SEELP_Value);
						sx1278_write_reg(REG_LR_DIOMAPPING1, 0X00);
						sx1278_write_reg(REG_LR_DIOMAPPING2, 0X00);
						sx1278_set_opmode(SLEEP);
					}
				} else {
					sx1278_set_opmode(STANDBY);
					sx1278_write_reg(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);
					sx1278_write_reg(REG_LR_HOPPERIOD, PACKET_MIAX_Value);
					sx1278_write_reg(REG_LR_DIOMAPPING1, 0X02);
					sx1278_write_reg(REG_LR_DIOMAPPING2, 0x00);
					sx1278_set_opmode(RX);
				}

				sx1278_write_reg(REG_LR_IRQFLAGS, 0xff);
				_delay_ms(300);
			}
#endif
		}
	}
	return 0;
}
