/*
 *  Copyright (c) 2019 - 2029 MaiKe Labs
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

#include "Arduino.h"
#include "U8g2lib.h"

#ifdef EFM32GG230F512
/*
 * PIN09_PC0: I2C0_SDA#4
 * PIN10_PC1: I2C0_SCL#4
*/

#define SH1107_RESET				2		/* PIN03_PA02_D2 */
#define	PWR_CTRL_PIN				5		/* PIN06_PA05_D5 */

#else
/*
 * EFM32ZG110F32 & EFM32HG110F64:
 *
 *  PIN23_PE12_D12: I2C0_SDA#6
 *  PIN24_PE13_D13: I2C0_SCL#6
 *
 *  PIN23_PE12_D12 - SH1107_SDA_PIN9
 *  PIN24_PE13_D13 - SH1107_SCL_PIN10
 *  PIN21_PF02_D16 - SH1107_RST_PIN14
 *
 *  GND - SH1107_A0_PIN13 (I2C_ADDR = 0x78)
 *
 */
#define SH1107_SDA					12
#define SH1107_SCL					13
#define SH1107_RESET				16

#define	PWR_CTRL_PIN				8		/* PIN17_PC14_D8 */
#endif

U8G2_SH1107_SEEED_128X128_1_HW_I2C u8g2(U8G2_R0, SH1107_RESET);

void draw(void)
{
	u8g2.setFont(u8g2_font_freedoomr10_mu);
	u8g2.drawStr(23, 60, "HELLO WORLD!");
}

void setup(void)
{
	// init dev power ctrl pin
	pinMode(PWR_CTRL_PIN, OUTPUT);

	// power on
	digitalWrite(PWR_CTRL_PIN, HIGH);

	u8g2.begin();
}

void loop(void)
{
	u8g2.setPowerSave(0);

	u8g2.firstPage();
	do {
		draw();
	} while (u8g2.nextPage());

	draw();

	delay(5000);
}
