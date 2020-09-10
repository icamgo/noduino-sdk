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

/*
 * make sure the payload len in sx127x.h:
 *  MAX_LENGTH = 48;
 *  MAX_PAYLOAD = 44;
*/
#define ENABLE_RX_INTERRUPT		1

#ifdef ENABLE_RX_INTERRUPT
#include "circ_buf.h"
struct circ_buf g_cbuf;
#define KEY_LONG_PRESS_TIME		60000

#else
#define KEY_LONG_PRESS_TIME		8
#endif

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

#define SYNCWORD_DEFAULT		0x12
#define SYNCWORD_LORAWAN		0x34
#define SYNCWORD_ABC			0x55

// use the dynamic ACK feature of our modified SX1272 lib
//#define GW_AUTO_ACK

#ifdef CONFIG_V0

#define RECEIVE_ALL

#define TXRX_CH				CH_01_472
//#define TXRX_CH				CH_00_470
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

// be careful, max command length is 60 characters
#define MAX_CMD_LENGTH			100
char cmd[MAX_CMD_LENGTH];

// number of retries to unlock remote configuration feature
bool withAck = false;

int status_counter = 0;
uint8_t rx_err_cnt = 0;

/*
 * Output Mode:
 *
 *   0x0: rx all messages
 *   0x1: only rx trigged message
 *   0x2: show the raw message
 *   0x3: only rx the tagged message
*/
#define	MODE_ALL		0
#define	MODE_KEY		1
#define	MODE_RAW		2
#define	MODE_ABC		3

#define NUM_MODE		4

#ifdef DEBUG_HEX_PKT
int omode = MODE_RAW;
int old_omode = MODE_RAW;
#else
int omode = MODE_KEY;
int old_omode = MODE_KEY;
#endif

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

#endif

#endif

void radio_setup()
{

#ifdef CONFIG_V0
	sx1272.setup_v0(TXRX_CH, 20);
	//sx1272.setPreambleLength(6);

	#ifdef ENABLE_ABC
	if (MODE_ABC == omode) {
		sx1272.setSyncWord(SYNCWORD_ABC);
	}
	#endif
#else
	sx1272.sx1278_qsetup(TXRX_CH, 20);
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
		case 0x34:
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
		//dd = (float)(data / 10.0);
		//ftoa(dev_data, dd, 0);
		//sprintf(dev_data, "%s %d", dev_data, (int8_t)(pkt[20]));
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
		case 0x34:
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

	//u8g2.clearBuffer();		// clear the internal memory

	//u8g2.setFont(u8g2_font_logisoso16_tf);	// choose a suitable font
	//u8g2.setFont(u8g2_font_sirclivethebold_tr);

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
		} else if (MODE_ABC == mode) {
			// Bell icon. notice the message tagged for testing
		#ifdef ENABLE_ABC
			u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
			if (l == 0) {
				u8g2.drawGlyph(120, 12, 65);
			} else if (l == 1) {
				u8g2.drawGlyph(120, 30, 65);
			}
		#endif
		} else if (MODE_KEY == mode) {
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
			u8g2.print(" T2 - ALL ");

			u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
			u8g2.drawGlyph(112, 23, 81);
		} else if (MODE_KEY == mode) {
			// cycle icon. notice the message trigged by magnet
			u8g2.setFont(u8g2_font_freedoomr10_mu);	// choose a suitable font
			u8g2.setCursor(12, 26);
			u8g2.print(" T2 - KEY ");

			u8g2.setFont(u8g2_font_open_iconic_app_1x_t);
			u8g2.drawGlyph(112, 23, 64);
		} else if (MODE_RAW == mode) {
			// Lora icon. notice rx all raw message
			u8g2.setFont(u8g2_font_freedoomr10_mu);	// choose a suitable font
			u8g2.setCursor(12, 64);
			u8g2.print(" T2 - RAW ");

			u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
			u8g2.drawGlyph(112, 61, 81);
		} else if (MODE_ABC == mode) {
			// Bell icon. notice the message tagged for testing
			u8g2.setFont(u8g2_font_freedoomr10_tu);	// choose a suitable font
			u8g2.setCursor(12, 26);
		#ifdef ENABLE_ABC
			u8g2.print(" T3 - ABC ");
		#else
			u8g2.print(" T2 - EXP ");
		#endif

			u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
			u8g2.drawGlyph(112, 23, 65);
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
	omode %= NUM_MODE;
	INFO("%s", "omode: ");
	INFOLN("%d", omode);
}

#ifdef ENABLE_ABC
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
#endif

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

#if 0
	switch (p[2]) {

		case 0x31:
			len = 15;
			sum = p[15] << 8 | p[16];
			break;

		case 0x32:
			len = 17;
			sum = p[17] << 8 | p[18];
			break;

		case 0x33:
			len = 18;
			sum = p[18] << 8 | p[19];
			break;
		default:
			len = 0;
			sum = 1;
			break;
	}
#endif
#if 0
	switch (plen) {
		case 24:
			len = 18;
			sum = p[18] << 8 | p[19];
			break;
		case 23:
			len = 17;
			sum = p[17] << 8 | p[18];
			break;
		case 21:
			len = 15;
			sum = p[15] << 8 | p[16];
			break;
		default:
			len = 0;
			sum = 1;
			break;
	}
#else
	len = plen - 6;
	sum = p[len] << 8 | p[len+1];
#endif


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
	if (p[0] != 0x47 || p[1] != 0x4F) return false;

	if (p[2] < 0x31 || p[2] > 0x35) return false;

	if (check_crc(p, len) == false) return false;

	return true;
}

void rx_irq_handler()
{
	noInterrupts();
#ifdef ENABLE_RX_INTERRUPT
	int8_t e = 0;

	sx1272.getRSSIpacket();
	e = sx1272.get_pkt_v0();

	if (!e) {

		uint8_t plen = sx1272._payloadlength;
		uint8_t *p = sx1272.packet_received.data;

		if (plen > PKT_LEN) return;

		if (omode == MODE_RAW ||
			(omode == MODE_KEY && is_our_pkt(p, plen) && (p[15]&0x0F) >= 3) ||
			((omode == MODE_ABC || omode == MODE_ALL) && is_our_pkt(p, plen))) {

			push_pkt(&g_cbuf, p, sx1272._RSSIpacket, plen);

		}

	} else {

		rx_err_cnt++;

	}
	//INFOLN("%d", e);

	//sx1272.rx_v0();
#endif
	interrupts();
}

void setup()
{
	int e;

	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = wdogPeriod_2k;	/* 2k 1kHz periods should give 2 seconds */

#ifndef DEBUG_HEX_PKT
	// Key connected to D0
	pinMode(KEY_PIN, INPUT);
	attachInterrupt(KEY_PIN, change_omode, FALLING);
#endif

#ifdef ENABLE_RX_INTERRUPT
	// RF RX Interrupt pin
	pinMode(RX_INT_PIN, INPUT);
	attachInterrupt(RX_INT_PIN, rx_irq_handler, RISING);
#endif

#ifdef ENABLE_ABC
	// beep
	pinMode(BEEP_PIN, OUTPUT);
	digitalWrite(BEEP_PIN, LOW);
#endif

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

#ifdef ENABLE_RX_INTERRUPT
	sx1272.init_rx_int();
	sx1272.rx_v0();
#endif

	/* Start watchdog */
	WDOG_Init(&wInit);
}

void loop(void)
{
	int e = 1;
	static int c = 0;

	WDOG_Feed();

#ifdef ENABLE_RX_INTERRUPT
	status_counter++;

	if (rx_err_cnt > 50) {

		sx1272.reset();
		INFO_S("%s", "Resetting lora module\n");
		radio_setup();

		sx1272.init_rx_int();
		sx1272.rx_v0();

		rx_err_cnt = 0;
	}

#endif

	if (omode != old_omode) {
#ifdef ENABLE_OLED
		// show mode and clean buffer
		show_mode(omode);

		frame_buf[0][0] = '\0';
		frame_buf[1][0] = '\0';
#endif
		old_omode = omode;

#ifdef ENABLE_ABC
		switch (omode) {
			case MODE_ALL:
				sx1272.setSyncWord(SYNCWORD_DEFAULT);
				break;
			case MODE_ABC:
				#ifdef ENABLE_ABC
				sx1272.setSyncWord(SYNCWORD_ABC);
				#endif
				//NVIC_SystemReset();
				break;
			case MODE_KEY:
				sx1272.setSyncWord(SYNCWORD_DEFAULT);
				break;
		}
#endif
	}

#ifdef ENABLE_OLED
	if (digitalRead(KEY_PIN) == 0) {
		// x 200ms
		key_time++;
	} else {
		key_time = 0;
	}

	if (key_time > KEY_LONG_PRESS_TIME) {

		key_time = 0;

		INFOLN("%s", "Key long time pressed, enter deep sleep...");
		delay(300);

		u8g2.setPowerSave(1);
		sx1272.setSleepMode();
		digitalWrite(SX1272_RST, LOW);

		spi_end();
		digitalWrite(10, LOW);
		digitalWrite(11, LOW);
		digitalWrite(12, LOW);
		digitalWrite(13, LOW);

		// dev power off
		power_off_dev();

#ifdef ENABLE_SH1106
		//wire_end();
		pinMode(SH1106_SCL, INPUT);
		pinMode(SH1106_SDA, INPUT);
		digitalWrite(SH1106_RESET, LOW);
#endif

		EMU_EnterEM2(true);

		setup();
	}
#endif

#if DEBUG > 1
	if (status_counter == 60 || status_counter == 0) {
		//INFO_S("%s", "^$Low-level gw status ON\n");
		INFO_S("%s", ".");
		status_counter = 0;

		if (adc.readVbat() < 2.92) {
			show_low_bat();
			delay(2700);
		}
	}
#endif

#ifndef ENABLE_RX_INTERRUPT
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

		uint8_t *p = sx1272.packet_received.data;
		uint8_t p_len = sx1272.getPayloadLength();

		int p_rssi = sx1272._RSSIpacket;
#else
	{
		struct pkt d;

		noInterrupts();
		int ret = get_pkt(&g_cbuf, &d);
		interrupts();

		if (ret != 0) return;

		uint8_t *p = d.data;
		uint8_t p_len = d.plen;
		int p_rssi = d.rssi;
#endif

#ifdef DEBUG_HEX_PKT
		int a = 0, b = 0;

		for (; a < p_len; a++, b++) {

			if ((uint8_t) p[a] < 16)
				INFO_S("%s", "0");

			INFO_HEX("%X", (uint8_t) p[a]);
			INFO_S("%s", " ");
		}

		if (p_len > 0) {
			INFO_S("%d", "/");
			INFO("%d", p_rssi);
			INFO_S("%s", "/");
			INFOLN("%d", p_len);
		}
#endif

#ifndef DEBUG_HEX_PKT
		if (p_len <= 11) return;
#endif
		status_counter = 0;

		memset(p+p_len, 0, MAX_PAYLOAD-p_len);
		decode_devid(p);

#ifndef ENABLE_RX_INTERRUPT
		if (strcmp(dev_id, "") == 0) {
			// lora module unexpected error
			sx1272.reset();

			INFO_S("%s", "Resetting lora module\n");

			radio_setup();
		}
#endif

		if (MODE_ALL == omode) {
			// show all message
			if (p[0] != 0x47 || p[1] != 0x4F) {
				return;
			}

			sprintf(cmd, "%s/U/%s/%s/%s/c/%d/v/%d/rssi/%d",
				dev_id,
				decode_vbat(p),
				decode_sensor_type(),
				decode_sensor_data(p),
				decode_cmd(p),
				decode_ver(p),
				p_rssi);

#ifdef ENABLE_OLED
				sprintf(frame_buf[c % 2], "%s %4d",
					dev_id,
					d.rssi);

				show_frame(c % 2, omode, false);
				c++;
#endif

				INFOLN("%s", cmd);
		} else if (MODE_ABC == omode) {
	#ifdef ENABLE_ABC
			// only show tagged message
			if (p[0] == 0x55) {

				sprintf(cmd, "%s/U/%s/%s/%s/rssi/%d",
					dev_id,
					decode_vbat(p),
					decode_sensor_type(),
					decode_sensor_data(p),
					d.rssi);

			#ifdef ENABLE_OLED
				sprintf(frame_buf[c % 2], " %c  %4d  %s",
					did2abc(p[1]),
					d.rssi,
					decode_vbat(p));

				show_frame(c % 2, omode, false);
				c++;
			#endif

				beep(p[1], 150 + d.rssi);

				INFOLN("%s", cmd);
	#else
			// show exception message
			if (p[0] == 0x47 && p[1] == 0x4F && (p[3] != 0 || p[4] != 0 || p[5] != 0
				|| (p[2] == 0x31 && p_len != 21) || (p[2] == 0x32 && p_len != 23)
				|| check_crc(p, p_len) == false) || p[2] > 0x36 || p[2] < 0x31) {

				sprintf(cmd, "%s/rssi/%d",
					dev_id,
					p_rssi);

			#ifdef ENABLE_OLED
				sprintf(frame_buf[0], "%s", dev_id);

				sprintf(frame_buf[1], "%02X %d %d %s%4d",
					p[2],
					p_len,
					check_crc(p, p_len),
					decode_vbat(p),
					p_rssi);

				show_frame(0, omode, false);
				c++;
			#endif
				INFOLN("%s", cmd);
	#endif
			}
		} else if (MODE_KEY == omode) {
			// only show trigged message

			if (p[0] != 0x47 || p[1] != 0x4F) {
				return;
			}

			if ((p[2] == 0x33 || p[2] == 0x34) && (p[15] == 0x03 || p[15] == 0x04 || p[15] == 0x05)) {

				sprintf(cmd, "%s/U/%s/%s/%s/rssi/%d",
					dev_id,
					decode_vbat(p),
					decode_sensor_type(),
					decode_sensor_data(p),
					p_rssi);

#ifdef ENABLE_OLED
				sprintf(frame_buf[0], "%s %4d",
					dev_id,
					d.rssi);

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
				c++;
#endif

				INFOLN("%s", cmd);
			}

		} else if (MODE_RAW == omode) {

			// only show raw message, <= 32bytes
#ifdef ENABLE_OLED
				sprintf(frame_buf[0], "%02X%02X%02X%02X%02X%02X%02X%02X%02X",
					p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]);

				sprintf(frame_buf[1], "%02X%02X %d %d %s%4d",
					p[9], p[10], p_len, check_crc(p, p_len), decode_vbat(p), d.rssi);

				show_frame(0, omode, false);
#endif
		}
	}
}
