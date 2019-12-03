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

#define	DEBUG					1

#define ENABLE_SSD1306			1

// use the dynamic ACK feature of our modified SX1272 lib
//#define GW_AUTO_ACK

#ifdef CONFIG_V0

#define RECEIVE_ALL

#define TXRX_CH				CH_01_472
#define RX_TIME				300

uint8_t loraMode = 12;

#else

#define TXRX_CH				CH_00_470		// 470.0MHz
#define RX_TIME				MAX_TIMEOUT

// Default LoRa mode BW=125KHz, CR=4/5, SF=12
uint8_t loraMode = 11;

// Gateway address: 1
uint8_t loraAddr = 1;

#endif

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
int omode = 0;
int old_omode = 0;

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
#else
	sx1272.sx1278_qsetup(CH_00_470, 20);
	sx1272._nodeAddress = loraAddr;

	sx1272._enableCarrierSense = true;
#endif
}

#ifdef CONFIG_V0
char goid_buf[21];
char go_vbat[6];
char go_temp[10];

char *uint64_to_str(uint64_t n)
{
	char *dest;
	dest = goid_buf;

	dest += 20;
	*dest-- = 0;
	while (n) {
		*dest-- = (n % 10) + '0';
		n /= 10;
	}
	return dest + 1;
}

char *decode_goid(uint8_t *pkt)
{
	int a = 0, b = 0;

	uint64_t goid = 0UL;

	for (a = 3; a < 11; a++, b++) {

		*(((uint8_t *)&goid) + 7 - b) = pkt[a];
	}

	return uint64_to_str(goid);
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

	ftoa(go_vbat, (float)(vbat / 1000.0), 2);
}

char *decode_temp(uint8_t *pkt)
{
	int16_t temp = 0;

	temp = ((pkt[11] << 8) & 0x7F) | pkt[12];

	if (pkt[11] & 0x80)
		temp = temp * -1;

	ftoa(go_temp, (float)(temp / 10.0), 1);
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
#endif

#ifdef ENABLE_SSD1306
void show_frame(int l, int mode)
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
			u8g2.setFont(u8g2_font_open_iconic_app_1x_t);
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
		u8g2.setFont(u8g2_font_profont29_tr);	// choose a suitable font
		u8g2.setCursor(4, 26);
		u8g2.print("AutoHeat");
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
			u8g2.setCursor(10, 28);
			u8g2.print("RX ALL MSG");

			u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
			u8g2.drawGlyph(116, 24, 81);
		} else if (1 == mode) {
			// Bell icon. notice the message tagged for testing
			u8g2.setFont(u8g2_font_freedoomr10_tu);	// choose a suitable font
			u8g2.setCursor(10, 28);
			u8g2.print("RX ABC MSG");

			u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
			u8g2.drawGlyph(116, 24, 65);
		} else if (2 == mode) {
			// cycle icon. notice the message trigged by magnet
			u8g2.setFont(u8g2_font_freedoomr10_mu);	// choose a suitable font
			u8g2.setCursor(10, 28);
			u8g2.print("RX TRIG MSG");

			u8g2.setFont(u8g2_font_open_iconic_app_1x_t);
			u8g2.drawGlyph(116, 24, 64);
		}
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

void beep(int c, int ontime)
{
	switch (c) {
		case 3:
			digitalWrite(7, HIGH);
			delay(ontime);
			digitalWrite(7, LOW);
			delay(40);
		case 2:
			digitalWrite(7, HIGH);
			delay(ontime);
			digitalWrite(7, LOW);
			delay(40);
		case 1:
			digitalWrite(7, HIGH);
			delay(ontime);
			digitalWrite(7, LOW);
	}
}

void setup()
{
	int e;

	// Key connected to D2
	pinMode(2, INPUT_PULLUP);

	// attach interrupt in D2
	attachInterrupt(0, change_omode, FALLING);

	// beep
	pinMode(7, OUTPUT);
	digitalWrite(7, LOW);

	Serial.begin(115200);

	// turn on the device power
#ifdef ENABLE_SSD1306
	// open-plant use the D6 to ctrl dev pwr
	pinMode(6, OUTPUT);
	digitalWrite(6, HIGH);
#else
	// quark v1.0 use the D7
	pinMode(7, OUTPUT);
	digitalWrite(7, LOW);
#endif

	radio_setup();

#ifdef ENABLE_SSD1306
	u8g2.begin();
#endif

	show_logo();
	delay(800);
	show_mode(omode);
}

void loop(void)
{
	int e = 1;
	static int c = 0;

	if (omode != old_omode) {
		// show mode and clean buffer
		show_mode(omode);
		old_omode = omode;

		frame_buf[0][0] = '\0';
		frame_buf[1][0] = '\0';
	}

	if (status_counter == 60 || status_counter == 0) {
		//INFO_S("%s", "^$Low-level gw status ON\n");
		INFO_S("%s", ".");
		status_counter = 0;
	}

	// check if we received data from the receiving LoRa module
#ifdef RECEIVE_ALL
	e = sx1272.receiveAll(RX_TIME);
#else
#ifdef GW_AUTO_ACK

	e = sx1272.receivePacketTimeout(RX_TIME);

	status_counter++;

	if (e != 0 && e != 3) {
		INFO_S("%s", "^$Receive error ");
		INFOLN("%d", e);

		if (e == 2) {
			// Power OFF the module
			sx1272.OFF();
			INFO_S("%s", "^$Resetting radio module\n");

			radio_setup();

			// to start over
			status_counter = 0;
			e = 1;
		}
		FLUSHOUTPUT;
	}

	if (!e && sx1272._requestACK_indicator) {
		INFO_S("%s", "^$ACK requested by ");
		INFOLN("%d", sx1272.packet_received.src);
		FLUSHOUTPUT;
	}
#else
	// OBSOLETE normally we always use GW_AUTO_ACK
	// Receive message
	if (withAck)
		e = sx1272.receivePacketTimeoutACK(RX_TIME);
	else
		e = sx1272.receivePacketTimeout(RX_TIME);

#endif // gw_auto_ack
#endif // receive_all

	if (!e) {

		sx1272.getRSSIpacket();

#ifdef CONFIG_V0
		INFOLN("%s", "");
		/*
		for (; a < p_len; a++, b++) {

			if ((uint8_t) sx1272.packet_received.data[a] < 16)
				INFO_S("%s", "0");

			INFO_HEX("%X", (uint8_t) sx1272. packet_received.data[a]);
			INFO_S("%s", " ");
		}

		INFOLN("%d", "$");
		*/

		uint8_t *p = sx1272.packet_received.data;

		if (0x0 == omode) {
			// show all message
			sprintf(cmd, "%s/U/%s/T/%s/c/%d/v/%d/rssi/%d",
				decode_goid(sx1272.packet_received.data),
				decode_vbat(sx1272.packet_received.data),
				decode_temp(sx1272.packet_received.data),
				decode_cmd(sx1272.packet_received.data),
				decode_ver(sx1272.packet_received.data),
				sx1272._RSSIpacket);

				sprintf(frame_buf[c % 2], "%s %4d",
					decode_goid(sx1272.packet_received.data),
					sx1272._RSSIpacket);

				show_frame(c % 2, omode);
				c++;

				INFOLN("%s", cmd);
		} else if (0x1 == omode) {
			// only show tagged message
			if (p[0] == 0x55 && p[1] == 0xaa) {
				sprintf(cmd, "%s/U/%s/T/%s/rssi/%d",
					decode_goid(sx1272.packet_received.data),
					decode_vbat(sx1272.packet_received.data),
					decode_temp(sx1272.packet_received.data),
					sx1272._RSSIpacket);

				sprintf(frame_buf[c % 2], "%s %4d",
					decode_goid(sx1272.packet_received.data),
					sx1272._RSSIpacket);

				show_frame(c % 2, omode);
				c++;

				INFOLN("%s", cmd);
			}
		} else if (0x2 == omode) {
			// only show trigged message
			if (p[2] == 0x33 && p[15] == 0x03) {
				sprintf(cmd, "%s/U/%s/T/%s/rssi/%d",
					decode_goid(sx1272.packet_received.data),
					decode_vbat(sx1272.packet_received.data),
					decode_temp(sx1272.packet_received.data),
					sx1272._RSSIpacket);

				sprintf(frame_buf[c % 2], "%s %4d",
					decode_goid(sx1272.packet_received.data),
					sx1272._RSSIpacket);

				show_frame(c % 2, omode);
				c++;

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
