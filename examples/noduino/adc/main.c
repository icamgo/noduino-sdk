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

/*
 * A0 --> 220K ---> 100K --> GND
 *              |
 *              |
 *           esp_adc
 */
void setup()
{
	serial_begin(115200);
	wifi_set_opmode(NULL_MODE);
}
 
void loop() {

  int ad = analogRead(A0);

  serial_printf("ad = %d\r\n", ad);
 
  float V = (ad / 1024.0) * (220 + 100)/100;
  serial_printf("V*1000 = %d\r\n", (int)(V*1000));
 
  delay(5000);
}
