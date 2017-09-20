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
#include "softuart.h"

#ifdef USE_SW_UART
Softuart softuart;
#endif

static os_timer_t test_timer;

static httpd_t httpd = {
   .esp_conn = NULL,
   .port = 80
};

void rs485_init()
{
#ifdef USE_SW_UART
	Softuart_SetPinRx(&softuart,13);
	Softuart_SetPinTx(&softuart,14);
	Softuart_Init(&softuart,9600);
	//set pin 12 as output to control tx enable/disable of rs485
	pinMode(12, OUTPUT);
	Softuart_EnableRs485(&softuart, 12);
#else
	serial_begin(9600);
	pinMode(12, OUTPUT);

	digitalWrite(12, LOW);
#endif
}

void rs485_write(uint8_t *d, int len)
{
#ifdef USE_SW_UART
	Softuart_Putbuf(&softuart, d, len);
#else
	int i;

	digitalWrite(12, HIGH);

	for (i=0; i < len; i++) {
		serial_write(d[i]);
		serial_flush();
	}

	digitalWrite(12, LOW);
#endif
}

void rs485_read(uint8_t *d, int len)
{
#ifdef USE_SW_UART
	if(Softuart_Available(&softuart)) {
		Softuart_Readbuf(&softuart, d, len);
	}
#else
	int i = 0;
	uint8_t c;
	while (serial_available()) {
		c = serial_read();
		if (i < len) {
			d[i] = c;
			i++;
		} else {
			break;
		}
	}
#endif
}

irom void start_ap_mode()
{
	struct softap_config config;
	char ssid[] = "Noduino";

	// switch to softp ap mode
	wifi_set_opmode(SOFTAP_MODE);

	// get old softap config
	wifi_softap_get_config(&config);

	os_memcpy(config.ssid, ssid, os_strlen(ssid));
	config.ssid_len = os_strlen(ssid);

	// set the new ssid
	wifi_softap_set_config(&config);
}

void setup(void)
{
	serial1_begin(115200);
	delay(2);

	serial1_printf("http2modbus sketch!\r\n");

	rs485_init();

	start_ap_mode();
	httpd_start(&httpd);
}

void loop()
{
	//serial1_printf("Hello world!\r\n");
	delay(2000);
}
