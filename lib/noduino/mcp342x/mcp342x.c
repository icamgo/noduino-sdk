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
#include "mcp342x.h"

/*
 * 12 bit ADC, res = 0
 * 14 bit ADC, res = 1
 * 16 bit ADC, res = 2
 * 18 bit ADC, res = 3
 */
static uint8_t res = 3;
static uint32_t divisor;

static uint8_t adc_cfg = (MCP342X_START			\
						| MCP342X_CHANNEL_1		\
						| MCP342X_CONTINUOUS	\
						| MCP342X_GAIN_X1		\
						| MCP342X_18_BIT);

irom uint8_t mcp342x_read(int32_t *data)
{
   // pointer used to form int32 data
   uint8_t *p = (uint8_t *)data;
   int i;

   // timeout - not really needed?
   uint32_t start = millis();
   if ((adc_cfg & MCP342X_RES_FIELD) == MCP342X_18_BIT)  // in 18 bit mode?
   {
      do {   // 18-bit mode
         wire_requestFrom(MCP342X_ADDRESS, 4);
         if (wire_available() != 4) {
            INFO("read failed");
            return false;
         }
         for (i = 2; i >= 0; i--) {
            p[i] = wire_read();
         }

         // extend sign bits
         p[3] = p[2] & 0X80 ? 0XFF : 0;

         // read config/status byte
         uint8_t s = wire_read();
         if ((s & MCP342X_BUSY) == 0) return true;  // escape here
      } while (millis() - start < 500);   // allows rollover of millis()

   } else {
      do {  // 12-bit to 16-bit mode
         wire_requestFrom(MCP342X_ADDRESS, 3);
         if (wire_available() != 3) {
            INFO("read failed");
            return false;
         }
         p[1] = wire_read();
         p[0] = wire_read();
         // extend sign bits
         p[2] = p[1] & 0X80 ? 0XFF : 0;
         p[3] = p[2];
         // read config/status byte
         uint8_t s = wire_read();
         if ((s & MCP342X_BUSY) == 0) return true;  // or escape here
      } while (millis() - start < 500);   // allows rollover of millis()
   }
   INFO("read timeout");      // dang it
   return false;
}

irom int mcp342x_write(uint8_t d)
{
	int error = 0;
	wire_beginTransmission(MCP342X_ADDRESS);
	wire_write(d);
	
	error = wire_endTransmission();
	return error;
}

irom int mcp342x_set_cfg(uint8_t cfg)
{
	int e;
	e = mcp342x_write(cfg);
	return e;
}

irom void mcp342x_set_oneshot()
{
	adc_cfg &= (~MCP342X_CONTINUOUS);
}

irom void mcp342x_set_continuous()
{
	adc_cfg |= MCP342X_CONTINUOUS;
}

irom int mcp342x_get_uv()
{
	int32_t data;

	mcp342x_set_cfg(adc_cfg);

	if (!mcp342x_read(&data)) {
		INFO("mcp342x read failed!\r\n");
		return (2048000 + 10000);
	}

	INFO("data = %d\r\n", data);

	// voltage in millivolts
	double mv = ((double)data * 2048.0) / (double) divisor;

	// uncomment line below to convert reading to microvolts
	int uv = mv * 1000;

	return uv;
}

irom void mcp342x_init()
{
	wire_begin();

	mcp342x_set_cfg(adc_cfg);

	divisor = 1 << (11 + 2 * res);
}
