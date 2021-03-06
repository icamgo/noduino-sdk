/*
 *  Copyright (c) 2015 - 2025 MaiKe Labs
 *
 *  Scanning addresses changed from 1..127
 *  This sketch tests the standard 7-bit addresses
 *  Devices with higher bit address might not be seen properly.
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
#include "softi2c.h"
#include "sht2x.h"

#if 0
#define SDA_PIN					11		/* PIN14_PD7 */
#define SCL_PIN					16		/* PIN21_PF2 */
#else
#define SDA_PIN					12		/* PIN23_PE12 */
#define SCL_PIN					13		/* PIN24_PE13 */
#endif

char *ftoa(char *a, double f, int precision)
{
	long p[] =
	    { 0, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000 };

	char *ret = a;
	long heiltal = (long)f;
	itoa(heiltal, a, 10);
	while (*a != '\0')
		a++;
	*a++ = '.';
	long desimal = abs((long)((f - heiltal) * p[precision]));
	if (desimal < p[precision - 1]) {
		*a++ = '0';
	}
	itoa(desimal, a, 10);
	return ret;
}

void setup()
{
	Serial.setRouteLoc(1);
	Serial.begin(115200);

	sht2x_init(SCL_PIN, SDA_PIN);

	Serial.println("SHT2x testing start...");
}

void loop()
{
	float temp = 0.0, humi = 0.0;
	char pres_s[10];

	temp = sht2x_get_temp();
	humi = sht2x_get_humi();

	char temp_s[10], humi_s[10];

	ftoa(temp_s, temp, 2);
	ftoa(humi_s, humi, 0);

	Serial.print("Temperatur = ");
	Serial.print(temp_s);
	Serial.println(" C");

	Serial.print("Humidy = ");
	Serial.print(humi_s);
	Serial.println(" %");

	delay(3000);
}
