/* 
 *  Copyright (c) 2019 - 2029 MaiKe Labs
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "SX1272.h"

#include <stdio.h>
#include <stdint.h>

#include "U8g2lib.h"
#include "logo.h"
#include "vbat.h"

#include "LowPower.h"

#define	DEBUG					1
//#define DEBUG_HEX_PKT			1

#define ENABLE_SSD1306			1

// use the dynamic ACK feature of our modified SX1272 lib
//#define GW_AUTO_ACK

#ifdef CONFIG_V0

#define RECEIVE_ALL

#define TXRX_CH				CH_01_472
#define RX_TIME				330

uint8_t loraMode = 12;

#else

#define TXRX_CH				CH_00_470		// 470.0MHz
#define RX_TIME				MAX_TIMEOUT

// Default LoRa mode BW=125KHz, CR=4/5, SF=12
uint8_t loraMode = 11;

// Gateway address: 1
uint8_t loraAddr = 1;

#endif

#define PWR_CTRL_PIN			6
#define KEY_PIN					2
#define BEEP_PIN				7

#define SYNCWORD_DEFAULT		0x12
#define SYNCWORD_LORAWAN		0x34
#define SYNCWORD_ABC			0x55

// be careful, max command length is 60 characters
#define MAX_CMD_LENGTH			100
char cmd[MAX_CMD_LENGTH];

// number of retries to unlock remote configuration feature
bool withAck = false;

int status_counter = 0;

char frame_buf[2][24];

/*
 * Output Mode:
 *
 *   0x0: rx all messages
 *   0x1: only rx the tagged message
 *   0x2: only rx trigged message
*/
int omode = 1;
int old_omode = 1;

int key_time = 0;

#ifdef DEBUG

#define INFO_S(fmt,param)			Serial.print(F(param))
#define INFO_HEX(fmt,param)			Serial.print(param,HEX)
#define INFO(fmt,param)				Serial.print(param)
#define INFOLN(fmt,param)			Serial.println(param)
#define FLUSHOUTPUT					Serial.flush();

#else

#define INFO_S(fmt,param)
#define INFO(fmt,param)
#define INFOLN(fmt,param)
#define FLUSHOUTPUT

#endif

#ifdef ENABLE_SSD1306
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);
#endif

void radio_setup()
{

#ifdef CONFIG_V0
	sx1272.setup_v0(CH_01_472, 20);
	sx1272.setSyncWord(SYNCWORD_ABC);
#else
	sx1272.sx1278_qsetup(CH_00_470, 20);
	sx1272._nodeAddress = loraAddr;

	sx1272._enableCarrierSense = true;
#endif
}

#ifdef CONFIG_V0
char dev_id[24];
char dev_vbat[6] = "00000";
char dev_type[8];
char dev_data[10];

char *uint64_to_str(uint64_t n)
{
	char *dest = dev_id;

	dest += 20;
	*dest-- = 0;
	while (n) {
		*dest-- = (n % 10) + '0';
		n /= 10;
	}

	strcpy(dev_id, dest+1);

	return dest + 1;
}

char *decode_devid(uint8_t *pkt)
{
	int a = 0, b = 0;

	uint64_t devid = 0UL;

	for (a = 3; a < 11; a++, b++) {

		*(((uint8_t *)&devid) + 7 - b) = pkt[a];
	}

	return uint64_to_str(devid);
}

/* only support .0001 */
char *ftoa(char *a, float f, int preci)
{
	long p[] =
	    {0, 10, 100, 1000, 10000};

	char *ret = a;

	long ipart = (long)f;

	//INFOLN("%d", ipart);

	itoa(ipart, a, 10);		//int16, -32,768 ~ 32,767

	while (*a != '\0')
		a++;

	*a++ = '.';

	long fpart = abs(f * p[preci] - ipart * p[preci]);

	//INFOLN("%d", fpart);

	if (fpart > 0) {
		if (fpart < p[preci]/10) {
			*a++ = '0';
		}
		if (fpart < p[preci]/100) {
			*a++ = '0';
		}
		if (fpart < p[preci]/1000) {
			*a++ = '0';
		}
	}

	itoa(fpart, a, 10);
	return ret;
}

char *decode_vbat(uint8_t *pkt)
{
	uint16_t vbat = 0;

	switch(pkt[2]) {
		case 0x31:
		case 0x32:
		case 0x33:
			vbat = pkt[13] << 8 | pkt[14];
			break;
	}

	ftoa(dev_vbat, (float)(vbat / 1000.0), 3);
	return dev_vbat;
}

// after decode_devid()
char *decode_sensor_type()
{
	if (dev_id[3] == '0') {
		switch(dev_id[4]) {
			case '0':
				strcpy(dev_type, "GOT1K");
				break;
			case '1':
				strcpy(dev_type, "GOP");
				break;
			case '2':
				strcpy(dev_type, "T2");
				break;
			case '3':
				strcpy(dev_type, "T2P");
				break;
			case '4':
				strcpy(dev_type, "GOT100");
				break;
			case '6':
				strcpy(dev_type, "GOWKF");
				break;
			case '7':
				strcpy(dev_type, "T2P");
				break;
			case '8':
				strcpy(dev_type, "T2TH");
				break;
			case '9':
				strcpy(dev_type, "T2M");
				break;
		}
	} else if (dev_id[3] == '1' && dev_id[4] == '0') {

		strcpy(dev_type, "GOMA");

	} else if (dev_id[3] == '1' && dev_id[4] == '1') {

		strcpy(dev_type, "MBUS");

	} else if (dev_id[3] == '1' && dev_id[4] == '2') {

		strcpy(dev_type, "T2V");

	} else if (dev_id[3] == '1' && dev_id[4] == '3') {

		strcpy(dev_type, "T2W");

	} else if (dev_id[3] == '2' && dev_id[4] == '0') {

		strcpy(dev_type, "GOCC");

	} else if (dev_id[3] == '2' && dev_id[4] == '1') {

		strcpy(dev_type, "T3ABC");
	}
	return dev_type;
}

char *decode_sensor_data(uint8_t *pkt)
{
	int16_t data = 0;
	float dd  = 0;

	data = ((pkt[11] & 0x7F) << 8) | pkt[12];

	if (pkt[11] & 0x80)
		data = data * -1;

	if (dev_id[3] == '0' && (dev_id[4] == '2' || dev_id[4] == '0' || dev_id[4] == '4')) {
		// Temperature
		dd = (float)(data / 10.0);
		ftoa(dev_data, dd, 1);
		sprintf(dev_data, "%s ", dev_data);

	} else if (dev_id[3] == '0' && (dev_id[4] == '1' || dev_id[4] == '3' || dev_id[4] == '7')) {
		// Pressure
		dd = (float)(data / 100.0);
		ftoa(dev_data, dd, 2);
		sprintf(dev_data, "%s  ", dev_data);

	} else if (dev_id[3] == '0' && dev_id[4] == '9') {
		// Moving Sensor
		sprintf(dev_data, "%dMM", data);

	} else if (dev_id[3] == '1' && dev_id[4] == '3') {
		// Water Leak Sensor
		dd = (float)(data / 10.0);
		ftoa(dev_data, dd, 1);
		sprintf(dev_data, "%s ", dev_data);

	} else if (dev_id[3] == '2' && dev_id[4] == '1') {
		// Internal Temprature of ABC Sensor
		dd = (float)(data / 10.0);
		ftoa(dev_data, dd, 1);
		sprintf(dev_data, "%s", dev_data);
	}
	return dev_data;
}

uint8_t decode_cmd(uint8_t *pkt)
{
	uint8_t cmd = 0;

	switch(pkt[2]) {

		case 0x33:
			cmd = pkt[15];
			break;
		default:
			cmd = 255;
			break;
	}

	return cmd;
}

uint8_t decode_ver(uint8_t *pkt)
{
	return pkt[2];
}

char did2abc(uint8_t did)
{
	switch(did) {

		case 1:
			return 'A';
			break;
		case 2:
			return 'B';
			break;
		case 3:
			return 'C';
			break;
		default:
			return 'X';
	}
}
#endif

#ifdef ENABLE_SSD1306
void show_frame(int l, int mode, bool alarm)
{
	u8g2.setPowerSave(0);

	//u8g2.clearBuffer();		// clear the internal memory

	//u8g2.setFont(u8g2_font_logisoso16_tf);	// choose a suitable font
	//u8g2.setFont(u8g2_font_sirclivethebold_tr);

	u8g2.firstPage();

	do {
		//u8g2.drawStr(98, 26, "Pa");		// write something to the internal memory
		u8g2.setFont(u8g2_font_freedoomr10_tu);

		u8g2.setCursor(2, 15);
		u8g2.print(frame_buf[0]);

		u8g2.setCursor(2, 32);
		u8g2.print(frame_buf[1]);

		if (0 == mode) {
			// Lora icon. notice rx all message
			u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
			if (l == 0) {
				u8g2.drawGlyph(120, 12, 81);
			} else if (l == 1) {
				u8g2.drawGlyph(120, 30, 81);
			}
		} else if (1 == mode) {
			// Bell icon. notice the message tagged for testing
			u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
			if (l == 0) {
				u8g2.drawGlyph(120, 12, 65);
			} else if (l == 1) {
				u8g2.drawGlyph(120, 30, 65);
			}
		} else if (2 == mode) {
			// cycle icon. notice the message trigged by magnet
			int ic = 64;

			if (alarm == false) {
				u8g2.setFont(u8g2_font_open_iconic_app_1x_t);
			} else {
				u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
				ic = 71;
			}

			if (l == 0) {
				u8g2.drawGlyph(120, 12, 64);
			} else if (l == 1) {
				u8g2.drawGlyph(120, 30, 64);
			}
		}

	} while (u8g2.nextPage());

	//delay(2000);

	//u8g2.setPowerSave(1);

	//u8g2.sendBuffer();		// transfer internal memory to the display
}

void show_logo()
{
	u8g2.setPowerSave(0);

	u8g2.firstPage();

	do {
		u8g2.drawXBM(1, 4, logo_width, logo_height, logo_xbm);
	} while (u8g2.nextPage());
}

void show_mode(int mode)
{
	u8g2.setPowerSave(0);

	u8g2.firstPage();

	do {
		if (0 == mode) {
			// Lora icon. notice rx all message
			u8g2.setFont(u8g2_font_freedoomr10_mu);	// choose a suitable font
			u8g2.setCursor(12, 26);
			u8g2.print(" T2 - ALL ");

			u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
			u8g2.drawGlyph(112, 23, 81);
		} else if (1 == mode) {
			// Bell icon. notice the message tagged for testing
			u8g2.setFont(u8g2_font_freedoomr10_tu);	// choose a suitable font
			u8g2.setCursor(12, 26);
			u8g2.print(" T3 - ABC ");

			u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
			u8g2.drawGlyph(112, 23, 65);
		} else if (2 == mode) {
			// cycle icon. notice the message trigged by magnet
			u8g2.setFont(u8g2_font_freedoomr10_mu);	// choose a suitable font
			u8g2.setCursor(12, 26);
			u8g2.print(" T2 - KEY ");

			u8g2.setFont(u8g2_font_open_iconic_app_1x_t);
			u8g2.drawGlyph(112, 23, 64);
		}
	} while (u8g2.nextPage());
}

void show_low_bat()
{
	u8g2.setPowerSave(0);

	u8g2.firstPage();

	do {
		u8g2.setFont(u8g2_font_freedoomr10_mu);	// choose a suitable font
		u8g2.setCursor(12, 26);
		u8g2.print(" LOW BATTERY ");
	} while (u8g2.nextPage());
}
#endif

void change_omode()
{
	omode++;
	omode %= 3;
	INFO("%s", "omode: ");
	INFOLN("%d", omode);
}

// 10 ms, [20, 100]
void beep(int c, int ontime)
{
	switch (c) {
		case 3:
			digitalWrite(BEEP_PIN, HIGH);
			delay(ontime);
			digitalWrite(BEEP_PIN, LOW);
			delay(40);
		case 2:
			digitalWrite(BEEP_PIN, HIGH);
			delay(ontime);
			digitalWrite(BEEP_PIN, LOW);
			delay(40);
		case 1:
			digitalWrite(BEEP_PIN, HIGH);
			delay(ontime);
			digitalWrite(BEEP_PIN, LOW);
	}
}

void setup()
{
	int e;

	// Key connected to D2
	pinMode(KEY_PIN, INPUT_PULLUP);

	// attach interrupt in D2
	attachInterrupt(0, change_omode, FALLING);

	// beep
	pinMode(BEEP_PIN, OUTPUT);
	digitalWrite(BEEP_PIN, LOW);

	// turn on the device power
#ifdef ENABLE_SSD1306
	// open-plant use the D6 to ctrl dev pwr
	pinMode(PWR_CTRL_PIN, OUTPUT);
	digitalWrite(PWR_CTRL_PIN, HIGH);
#else
	// quark v1.0 use the D7
	pinMode(7, OUTPUT);
	digitalWrite(7, LOW);
#endif

#ifdef ENABLE_SSD1306
	u8g2.begin();

	delay(2);
	show_logo();
	delay(900);

	if (get_vbat() < 2.92) {
		// show low power
		show_low_bat();
		delay(2900);
	}

	show_mode(omode);
#endif

	Serial.begin(115200);

	radio_setup();
}

void loop(void)
{
	int e = 1;
	static int c = 0;

#ifdef ENABLE_SSD1306
	if (omode != old_omode) {
		// show mode and clean buffer
		show_mode(omode);
		old_omode = omode;

		switch (omode) {
			case 0:
				sx1272.setSyncWord(SYNCWORD_DEFAULT);
				break;
			case 1:
				sx1272.setSyncWord(SYNCWORD_ABC);
				//NVIC_SystemReset();
				break;
			case 2:
				sx1272.setSyncWord(SYNCWORD_DEFAULT);
				break;
		}

		frame_buf[0][0] = '\0';
		frame_buf[1][0] = '\0';
	}
#endif

	if (digitalRead(2) == 0) {
		// x 200ms
		key_time++;
	} else {
		key_time = 0;
	}

	if (key_time > 8) {

		key_time = 0;

		INFOLN("%s", "Key long time pressed, enter deep sleep...");
		delay(300);

#ifdef ENABLE_SSD1306
		u8g2.setPowerSave(1);
#endif
		sx1272.setSleepMode();
		digitalWrite(SX1272_RST, LOW);

		SPI.end();
		digitalWrite(10, LOW);
		digitalWrite(11, LOW);
		digitalWrite(12, LOW);
		digitalWrite(13, LOW);

		// dev power off
		digitalWrite(PWR_CTRL_PIN, LOW);

		LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

		setup();
	}

#if DEBUG > 1
	if (status_counter == 60 || status_counter == 0) {
		//INFO_S("%s", "^$Low-level gw status ON\n");
		INFO_S("%s", ".");
		status_counter = 0;

		if (get_vbat() < 2.92) {
			// show low power
			show_low_bat();
			delay(2900);
		}
	}
#endif

	// check if we received data from the receiving LoRa module
#ifdef RECEIVE_ALL
	e = sx1272.receiveAll(RX_TIME);
#else
	e = sx1272.receivePacketTimeout(RX_TIME);

	status_counter++;

	if (e != 0 && e != 3) {
		// e = 1 or e = 2 or e > 3
#if DEBUG > 1
		INFO_S("%s", "^$Receive error ");
		INFOLN("%d", e);
#endif

		if (e == 2) {
			// Power OFF the module
			sx1272.reset();
#if DEBUG > 1
			INFO_S("%s", "^$Resetting radio module\n");
#endif

			radio_setup();

			// to start over
			e = 1;
		}
	}
#endif

	if (!e) {

		sx1272.getRSSIpacket();

#ifdef CONFIG_V0

#if DEBUG > 1
		INFOLN("%s", "");
#endif

#ifdef DEBUG_HEX_PKT
		int a = 0, b = 0;
		uint8_t p_len = sx1272.getPayloadLength();

		for (; a < p_len; a++, b++) {

			if ((uint8_t) sx1272.packet_received.data[a] < 16)
				INFO_S("%s", "0");

			INFO_HEX("%X", (uint8_t) sx1272. packet_received.data[a]);
			INFO_S("%s", " ");
		}

		INFOLN("%d", "$");
#endif

		uint8_t *p = sx1272.packet_received.data;

		if (0x0 == omode) {
			// show all message
			decode_devid(sx1272.packet_received.data);

			sprintf(cmd, "%s/U/%s/%s/%s/c/%d/v/%d/rssi/%d",
				dev_id,
				decode_vbat(sx1272.packet_received.data),
				decode_sensor_type(),
				decode_sensor_data(sx1272.packet_received.data),
				decode_cmd(sx1272.packet_received.data),
				decode_ver(sx1272.packet_received.data),
				sx1272._RSSIpacket);

#ifdef ENABLE_SSD1306
				sprintf(frame_buf[c % 2], "%s %4d",
					dev_id,
					sx1272._RSSIpacket);

				show_frame(c % 2, omode, false);
				c++;
#endif

				INFOLN("%s", cmd);
		} else if (0x1 == omode) {
			// only show tagged message
			if (p[0] == 0x55) {

				decode_devid(sx1272.packet_received.data);

				sprintf(cmd, "%s/U/%s/%s/%s/rssi/%d",
					dev_id,
					decode_vbat(sx1272.packet_received.data),
					decode_sensor_type(),
					decode_sensor_data(sx1272.packet_received.data),
					sx1272._RSSIpacket);

#ifdef ENABLE_SSD1306
				sprintf(frame_buf[c % 2], " %c  %4d  %s",
					did2abc(p[1]),
					sx1272._RSSIpacket,
					decode_vbat(sx1272.packet_received.data));

				show_frame(c % 2, omode, false);
				c++;
#endif

				beep(p[1], 150 + sx1272._RSSIpacket);

				INFOLN("%s", cmd);
			}
		} else if (0x2 == omode) {
			// only show trigged message
			if (p[2] == 0x33 && (p[15] == 0x03 || p[15] == 0x04)) {

				decode_devid(sx1272.packet_received.data);

				sprintf(cmd, "%s/U/%s/%s/%s/rssi/%d",
					dev_id,
					decode_vbat(p),
					decode_sensor_type(),
					decode_sensor_data(p),
					sx1272._RSSIpacket);

#ifdef ENABLE_SSD1306
				sprintf(frame_buf[0], "%s %4d",
					dev_id,
					sx1272._RSSIpacket);

				sprintf(frame_buf[1], " %s %s %s",
					dev_type,
					dev_data,
					dev_vbat
					);

				show_frame(0, omode, p[15] & 0x04);
				c++;
#endif

				INFOLN("%s", cmd);
			}
		}

#else
		int a = 0, b = 0;
		uint8_t p_len;

		p_len = sx1272.getPayloadLength();

		for (; a < p_len; a++, b++) {

			cmd[b] = (char)sx1272.packet_received.data[a];
		}

		cmd[b] = '\0';

		b = strlen(cmd);

		// src_id,SNR,RSSI
		sprintf(cmd+b, "/devid/%d/rssi/%d/snr/%d",
			sx1272.packet_received.src,
			sx1272._RSSIpacket,
			sx1272._SNR);

		INFOLN("%s", cmd);
#endif
	}
}
