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

#include "softspi.h"
#include "sx1272.h"

#include <stdio.h>
#include <stdint.h>

#include "tx_ctrl.h"
#include "circ_buf.h"

struct circ_buf g_cbuf;
#define OLED_DELAY_TIME			3600000		/* oled is on about 35s */

struct ctrl_fifo g_cfifo;

#if 0
#define	DEBUG					1
#define DEBUG_HEX_PKT			1
#else
#define ENABLE_OLED					1
#define ENABLE_SH1106				1
#endif

//#define ENABLE_SSD1306			1

////////////////////////////////////////////////////////////
#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define	KEY_PIN					0		/* PIN01_PA00_D0 */
#define	BEEP_PIN				10		/* PIN13_PD06_D10 */
#define	RX_INT_PIN				3		/* PIN8_PB11_D3 */


#ifdef CONFIG_V0

//#define ENABLE_CAD				1
#define DEST_ADDR				1

#ifdef ENABLE_CAD
#define	TX_TIME					800		// 800ms
#else
#define	TX_TIME					220			// 220ms
#endif

#define TXRX_CH					CH_01_472

#endif

#ifdef DEBUG
#define MAX_CMD_LENGTH			100
char cmd[MAX_CMD_LENGTH];
#endif

int status_counter = 0;
uint8_t rx_err_cnt = 0;

uint32_t rx_cnt = 0;
uint32_t tx_cnt = 0;

/*
 * Output Mode:
 *
 *   0x0: rx all messages
 *   0x1: show the raw message
*/
#define MODE_NUM		4

#define	MODE_ALL		0
#define	MODE_RAW		1
#define	MODE_DECODE		2
#define MODE_STATIS		3

int omode = MODE_STATIS;
int old_omode = MODE_STATIS;

bool oled_on = true;

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

#ifdef ENABLE_OLED
#include "U8g2lib.h"
#include "logo.h"

char frame_buf[2][24];

#ifdef ENABLE_SSD1306
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);
#endif

#ifdef ENABLE_SH1106
/*
 * PIN24_PE13_D13 - SCL
 * PIN23_PE12_D12 - SDA
 * PIN21_PF02_D16 - Reset
 */
#define SH1106_SCL					13
#define SH1106_SDA					12
#define SH1106_RESET				16

#ifdef USE_SOFTI2C
U8G2_SH1106_128X32_NONAME_1_SW_I2C u8g2(U8G2_R2, SH1106_SCL, SH1106_SDA, SH1106_RESET);
#else
U8G2_SH1106_128X32_NONAME_1_HW_I2C u8g2(U8G2_R2, SH1106_RESET);
#endif

#endif // ENABLE_SH1106

#endif // ENABLE_OLED

void radio_setup()
{

#ifdef CONFIG_V0
	sx1272.setup_v0(CH_01_472, 20);
	//sx1272.setPreambleLength(6);
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
char dev_data[12];

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

	itoa(ipart, a, 10);		//int16, -32,768 ~ 32,767

	while (*a != '\0')
		a++;

	*a++ = '.';

	long fpart = abs(f * p[preci] - ipart * p[preci]);

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

	ftoa(dev_vbat, (float)(vbat / 1000.0), 1);
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
			case '5':
				strcpy(dev_type, "ETP");
				break;
			case '6':
				strcpy(dev_type, "EV");
				break;
			case '7':
				strcpy(dev_type, "T2WP");
				break;
			case '8':
				strcpy(dev_type, "HT");
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

		strcpy(dev_type, "T2WF");

	} else if (dev_id[3] == '1' && dev_id[4] == '4') {

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

	data = (pkt[11]  << 8) | pkt[12];

	if (dev_id[3] == '0' && (dev_id[4] == '2' || dev_id[4] == '0' || dev_id[4] == '4')) {
		// Temperature
		dd = (float)(data / 10.0);
		ftoa(dev_data, dd, 1);
		sprintf(dev_data, "%s ", dev_data);

	} else if (dev_id[3] == '0' && (dev_id[4] == '1' || dev_id[4] == '3')) {

		// Pressure
		dd = (float)(data / 100.0);
		ftoa(dev_data, dd, 2);
		sprintf(dev_data, "%s  ", dev_data);

	} else if (dev_id[3] == '0' && dev_id[4] == '7') {

		// water level (pressure)
		dd = (float)(data / 100.0);
		ftoa(dev_data, dd, 2);
		sprintf(dev_data, "%sM", dev_data);

	} else if (dev_id[3] == '0' && dev_id[4] == '5') {
		// ET-Pump
		dd = data;
		sprintf(dev_data, "0x%X", data);

	} else if (dev_id[3] == '0' && dev_id[4] == '8') {
		// Humi&Temp Sensor
		sprintf(dev_data, "%d %d", (int)(data/10.0+0.5), (int8_t)(pkt[20]));

	} else if (dev_id[3] == '0' && dev_id[4] == '9') {
		// Moving Sensor
		sprintf(dev_data, "%dMM", data);

	} else if (dev_id[3] == '1' && dev_id[4] == '2') {
		// Vibration Sensor
		sprintf(dev_data, "%d", data);

	} else if (dev_id[3] == '1' && dev_id[4] == '3') {
		// Float & Temp Sensor
		dd = (float)(data / 10.0);
		ftoa(dev_data, dd, 1);
		sprintf(dev_data, "%s ", dev_data);

	} else if (dev_id[3] == '1' && dev_id[4] == '4') {
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

#ifdef ENABLE_OLED
void show_frame(int l, int mode, bool alarm)
{
	u8g2.setPowerSave(0);

	u8g2.firstPage();

	do {
		//u8g2.drawStr(98, 26, "Pa");		// write something to the internal memory
		u8g2.setFont(u8g2_font_freedoomr10_tu);

		u8g2.setCursor(0, 15);
		u8g2.print(frame_buf[0]);

		u8g2.setCursor(0, 32);
		u8g2.print(frame_buf[1]);

		if (MODE_ALL == mode) {
			// Lora icon. notice rx all message
			u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
			if (l == 0) {
				u8g2.drawGlyph(120, 12, 81);
			} else if (l == 1) {
				u8g2.drawGlyph(120, 30, 81);
			}
		} else if (MODE_DECODE == mode) {
			// cycle icon. notice the message trigged by magnet
			int ic = 64;

			if (alarm == false) {
				u8g2.setFont(u8g2_font_open_iconic_app_1x_t);

				u8g2.drawGlyph(120, 12, ic);
			} else {
				// show alarm icon
				#if 0
				// show exclamation icon
				u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
				ic = 71;
				#endif

				if (dev_id[3] == '0' && (dev_id[4] == '1' || dev_id[4] == '3'
					|| dev_id[4] == '7' || dev_id[4] == '8')) {
					// Pressure & Level, Humidity digital sensor, showing lightning icon
					u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
					ic = 67;

					u8g2.drawGlyph(116, 15, ic);

				} else if (dev_id[3] == '1' && dev_id[4] == '3') {
					// Water Leak sensor, showing water drop icon
					u8g2.setFont(u8g2_font_open_iconic_thing_1x_t);
					ic = 72;

					u8g2.drawGlyph(118, 12, ic);
				} else if (dev_id[3] == '1' && dev_id[4] == '2') {
					// Vibration sensor, showing pulse icon
					u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);

					ic = 70;

					u8g2.drawGlyph(116, 15, ic);
				}

			}
		}

	} while (u8g2.nextPage());
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
		if (MODE_ALL == mode) {
			// Lora icon. notice rx all message
			u8g2.setFont(u8g2_font_freedoomr10_mu);	// choose a suitable font
			u8g2.setCursor(12, 26);
			u8g2.print(" CC - ALL ");

			u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
			u8g2.drawGlyph(112, 23, 81);
		} else if (MODE_RAW == mode) {
			// Lora icon. notice rx all raw message
			u8g2.setFont(u8g2_font_freedoomr10_mu);	// choose a suitable font
			u8g2.setCursor(12, 64);
			u8g2.print(" CC - RAW ");

			u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
			u8g2.drawGlyph(112, 61, 81);

		} else if (MODE_STATIS == mode) {
			// cycle icon. notice the message trigged by magnet
			u8g2.setFont(u8g2_font_freedoomr10_mu);	// choose a suitable font
			u8g2.setCursor(12, 26);
			u8g2.print(" CC - STA ");

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
	omode %= MODE_NUM;

	oled_on = true;

	INFO("%s", "omode: ");
	INFOLN("%d", omode);
}

void power_on_dev()
{
	digitalWrite(PWR_CTRL_PIN, HIGH);
}

void power_off_dev()
{
	digitalWrite(PWR_CTRL_PIN, LOW);
}

bool check_crc(uint8_t *p, int plen)
{
	int i, len = 0;
	uint16_t hh = 0, sum = 0;

	len = plen - 6;
	sum = p[len] << 8 | p[len+1];

	for (i = 0; i < len; i++) {
		hh += p[i];
	}

	if (hh == sum)
		return true;
	else
		return false;
}

bool is_our_pkt(uint8_t *p, int len)
{

	if (len < 24) return false;

	if (p[0] != 0x47 || p[1] != 0x4F) {

		// invalide pkt header
		return false;
	}

	if (p[3] != 0 || p[4] != 0 || p[5] != 0 || p[6] > 0x15) {

		// invalid devid
		return false;
	}

	if (p[2] < 0x33 || p[2] > 0x36) {

		// support only the 0x33/34/35/36 version
		return false;
	}

	if (check_crc(p, len) == false) {

		return false;
	}

	return true;
}

uint16_t update_crc(uint8_t *p, int len)
{
	int i, pos = len - 6;;

	uint16_t hh = 0;

	for (i = 0; i < pos; i++) {
		hh += p[i];
	}

	uint8_t *x = (uint8_t *) &hh;

	p[pos] = x[1]; p[pos+1] = x[0];

	return hh;
}

bool process_pkt(uint8_t *p, int *len)
{
	p[0] = 0x49;

	// p[15] is the cmd type

	if (p[2] == 0x33) {

		//extend the 0x33

		if (*len == 24 || *len == 28) {
			//TODO: extend the pkt to 32Bytes, add the relay_cnt

			/* move the frame no. to p[24:25] */
			//p[24] = p[16];
			//p[25] = p[17];
			p[24] = p[*len-8];
			p[25] = p[*len-7];

			p[16] = 0; p[17] = 0; p[18] = 0; p[19] = 0;

			// p[26:27] as the crc
			// p[28:31] as the reserved crc
			p[28] = 0; p[29] = 0; p[30] = 0; p[31] = 0;

			*len = 32;
		}

		if (*len == 32) {

			if (p[15] & 0x80) {
				// new cc relayed pkt

				if (p[17] >= 5) {

					// cc relayed times
					return false;

				} else {
					p[17] += 1;
				}

			} else {
				// device pkt
				p[15] |= 0x80;		// mark as cc-relayed
				p[17] = 1;			// increment the cc-relayed-cnt
			}

		}

	}

	if (check_ctrl_fno(&g_cfifo, p, *len) == true) {

		update_crc(p, *len);

		return true;

	} else {
		return false;
	}
}

bool is_cc_ok(uint8_t *p, int len)
{
	if (p[2] == 0x33 && len == 32) {

		if (p[15] & 0x80) {

			// new cc relayed pkt

			if (p[17] < 5) {
				return true;
			} else {
				return false;
			}

		} else {
			// no cc relayed
			return true;
		}

	} else {

		// other version pkt
		return true;
	}
}

void rx_irq_handler()
{
	int8_t e = 0;

	sx1272.getRSSIpacket();

	e = sx1272.get_pkt_v0();

	if (!e) {

		uint8_t plen = sx1272._payloadlength;
		uint8_t *p = sx1272.packet_received.data;

		if (plen > 44) return;

		rx_cnt++;

		if (is_our_pkt(p, plen) && is_cc_ok(p, plen) &&
			is_pkt_in_ctrl(&g_cfifo, p, plen, seconds()) == false) {

			// push into the tx queue buffer
			push_pkt(&g_cbuf, sx1272.packet_received.data, sx1272._RSSIpacket, plen);
		}

	} else {

		rx_err_cnt++;

	}
}

int tx_pkt(uint8_t *p, int len)
{
	//radio_setup();

#ifdef ENABLE_CAD
	sx1272.CarrierSense();
#endif

	int e = sx1272.sendPacketTimeout(DEST_ADDR, p, len, TX_TIME);

	if (0 == e) {
		// send message succesful,
		INFOLN("%s", "TX OK");
	}

	return e;
}

void setup()
{
	int e;

	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = wdogPeriod_2k;	/* 2k 1kHz periods should give 2 seconds */

	// Key connected to D0
	pinMode(KEY_PIN, INPUT);
	attachInterrupt(KEY_PIN, change_omode, FALLING);

	// RF RX Interrupt pin
	pinMode(RX_INT_PIN, INPUT);
	attachInterrupt(RX_INT_PIN, rx_irq_handler, RISING);

	// dev power ctrl
	pinMode(PWR_CTRL_PIN, OUTPUT);

	power_on_dev();

#ifdef ENABLE_OLED
	u8g2.begin();

	delay(2);
	show_logo();
	delay(800);

	if (adc.readVbat() < 2.92) {
		show_low_bat();
		delay(2700);
	}

	show_mode(omode);
#endif

#ifdef DEBUG
	Serial.setRouteLoc(1);
	Serial.begin(115200);
#endif

	radio_setup();

	sx1272.init_rx_int();
	sx1272.rx_v0();

	/* Start watchdog */
	WDOG_Init(&wInit);
}

void loop(void)
{
	int e = 1;
	static int c = 0;

	WDOG_Feed();

	status_counter++;

	if (rx_err_cnt > 50) {

		sx1272.reset();
		INFO_S("%s", "Resetting lora module\n");
		radio_setup();

		sx1272.init_rx_int();
		sx1272.rx_v0();

		rx_err_cnt = 0;
	}

	if (omode != old_omode) {
#ifdef ENABLE_OLED
		// show mode and clean buffer
		show_mode(omode);

		frame_buf[0][0] = '\0';
		frame_buf[1][0] = '\0';
#endif
		old_omode = omode;
	}

	if (status_counter > OLED_DELAY_TIME) {

		status_counter = 0;

		INFOLN("%s", "Time is over, turn off the oled...");

		oled_on = false;

#ifdef ENABLE_OLED
		u8g2.setPowerSave(1);
#endif
	}

	struct pkt d;

	noInterrupts();
	int ret = get_pkt(&g_cbuf, &d);
	interrupts();

	if (ret != 0) return;

	uint8_t *p = d.data;
	int p_len = d.plen;

#ifdef DEBUG_HEX_PKT
	int a = 0, b = 0;

	for (; a < p_len; a++, b++) {

		if ((uint8_t) p[a] < 16)
			INFO_S("%s", "0");

		INFO_HEX("%X", (uint8_t) p[a]);
		INFO_S("%s", " ");
	}

	INFO_S("%d", "/");
	INFO("%d", sx1272._RSSIpacket);
	INFO_S("%s", "/");
	INFOLN("%d", p_len);
#endif


	if (oled_on == true) {

		memset(p+p_len, 0, MAX_PAYLOAD-p_len);
		decode_devid(p);

		if (MODE_ALL == omode) {
			// show all message
			if (p[0] != 0x47 || p[1] != 0x4F || p[2] != 0x33) {
				return;
			}

		#ifdef ENABLE_OLED
				sprintf(frame_buf[c % 2], "%s %4d",
					dev_id,
					d.rssi);

				show_frame(c % 2, omode, false);
				c++;
		#endif

		#ifdef DEBUG
			sprintf(cmd, "%s/U/%s/%s/%s/c/%d/v/%d/rssi/%d",
				dev_id,
				decode_vbat(p),
				decode_sensor_type(),
				decode_sensor_data(p),
				decode_cmd(p),
				decode_ver(p),
				d.rssi);

				INFOLN("%s", cmd);
		#endif

		} else if (MODE_RAW == omode) {

			// only show raw message, <= 32bytes
		#ifdef ENABLE_OLED
			sprintf(frame_buf[0], "%02X%02X%02X%02X%02X%02X%02X%02X%02X",
				p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]);

			sprintf(frame_buf[1], "%02X%02X %d %d %s%4d",
				p[9], p[10], p_len, check_crc(p, p_len), decode_vbat(p), d.rssi);

			show_frame(0, omode, false);
		#endif

		} else if (MODE_STATIS == omode) {

		#ifdef ENABLE_OLED
			sprintf(frame_buf[0], " RX: %4d", rx_cnt);
			sprintf(frame_buf[1], " TX: %4d", tx_cnt);

			show_frame(0, omode, false);
		#endif

		} else if (MODE_DECODE == omode) {

		#ifdef ENABLE_OLED
			sprintf(frame_buf[0], "%s %4d",
				dev_id,
				d.rssi);

			decode_vbat(p);
			decode_sensor_type();
			decode_sensor_data(p);

			if (dev_id[3] == '0' && (dev_id[4] == '8')) {

				sprintf(frame_buf[1], "%s %s %s",
					dev_type,
					dev_data,
					dev_vbat
					);
			} else {
				sprintf(frame_buf[1], " %s %s %s",
					dev_type,
					dev_data,
					dev_vbat
					);
			}

			show_frame(0, omode, p[15] & 0x04);
		#endif
		}
	}

	if (process_pkt(p, &p_len) == true) {

		tx_pkt(p, p_len);

		tx_cnt++;

		sx1272.rx_v0();
	}
}
