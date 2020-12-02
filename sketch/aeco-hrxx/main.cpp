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

#include "crypto.h"

#define ENABLE_RX_INTERRUPT		1

#ifdef ENABLE_RX_INTERRUPT
#include "circ_buf.h"
struct circ_buf g_cbuf;

uint8_t rx_err_cnt = 0;
uint8_t rx_hung_cnt = 0;
#define KEY_LONG_PRESS_TIME		2

#define RX_ERR_THRESHOLD		15

#else
#define KEY_LONG_PRESS_TIME		8
#endif

//#define	DEBUG					1
//#define DEBUG_HEX_PKT			1

#define ENABLE_OLED					1
#define ENABLE_SH1107				1

#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define	KEY_PIN					0		/* PIN01_PA00_D0 */
#define	BEEP_PIN				10		/* PIN13_PD06_D10 */
#define	RX_INT_PIN				3		/* PIN8_PB11_D3 */

#define SYNCWORD_DEFAULT		0x12
#define SYNCWORD_LORAWAN		0x34
#define SYNCWORD_ABC			0x55

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

#define MAX_CMD_LENGTH			48
char cmd[MAX_CMD_LENGTH];

int status_counter = 0;

char frame_buf[8][24];

/*
 * Output Mode:
 *
 *   0x0: rx all messages
 *   0x1: only rx trigged message
 *   0x2: only rx the tagged message
*/
#define	MODE_ALL		0
#define	MODE_KEY		1
#define	MODE_CC2		2
#define	MODE_MAC		3
#define	MODE_RAW		4

int omode = MODE_KEY;
int old_omode = MODE_KEY;

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

#ifdef ENABLE_SH1107
/*
 * PIN24_PE13_D13 - SH1107_SCL_PIN10
 * PIN23_PE12_D12 - SH1107_SDA_PIN9
 * PIN21_PF02_D16 - SH1107_RST_PIN14
 * 
 * GND - SH1107_A0_PIN13 (I2C_ADDR = 0x78)
 */
#define SH1107_SDA					12
#define SH1107_SCL					13
#define SH1107_RESET				16

#ifdef USE_SOFTI2C
U8G2_SH1107_SEEED_128X128_1_SW_I2C u8g2(U8G2_R0, SH1107_SCL, SH1107_SDA, SH1107_RESET);
#else
U8G2_SH1107_SEEED_128X128_1_HW_I2C u8g2(U8G2_R0, SH1107_RESET);
#endif

#endif

#endif

uint32_t rx_cnt = 0;

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
char dev_id[24];
char dev_vbat[6] = "00000";
char dev_type[8];
char dev_data[12];

char *uint64_to_str(uint64_t n)
{
	char *dest = dev_id;

	memset(dev_id, 0, 24);

	dest += 20;
	*dest-- = 0;
	while (n) {
		*dest-- = (n % 10) + '0';
		n /= 10;
	}

	strcpy(dev_id, dest+1);

	return dev_id;
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

char *decode_ccid(uint8_t *pkt)
{
	uint64_t devid = 0UL;
	uint8_t *id_p = (uint8_t *)&devid;

	id_p[4] = pkt[18];
	id_p[3] = pkt[19];
	id_p[2] = pkt[24];
	id_p[1] = pkt[25];
	id_p[0] = pkt[26];

	return uint64_to_str(devid);
}

uint32_t decode_cc2_epoch(uint8_t *p)
{
	uint32_t epoch = 0;
	uint8_t *ep = (uint8_t *)&epoch;

	ep[3] = p[20];
	ep[2] = p[21];
	ep[1] = p[22];
	ep[0] = p[23];

	return epoch;
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

uint32_t get_dev_type(uint8_t *p)
{
	uint64_t devid = 0UL;
	uint8_t *pd = (uint8_t *) &devid;

	for(int i = 0; i < 8; i++) {
		pd[7-i] = p[3+i];
	}

	uint64_t temp = devid / 1000000;
	uint32_t tt = (uint32_t)(temp / 100);
	return (uint32_t)(temp - tt * 100);
}

// after decode_devid()
char *decode_sensor_type(uint32_t dev_t)
{
	switch(dev_t) {
		case 0:
			strcpy(dev_type, "GOT1K");
			break;
		case 1:
			strcpy(dev_type, "GOP");
			break;
		case 2:
			strcpy(dev_type, "T2");
			break;
		case 3:
			strcpy(dev_type, "T2P");
			break;
		case 4:
			strcpy(dev_type, "GOT100");
			break;
		case 5:
			strcpy(dev_type, "ETP");
			break;
		case 6:
			strcpy(dev_type, "EV");
			break;
		case 7:
			strcpy(dev_type, "T2WP");
			break;
		case 8:
			strcpy(dev_type, "HT");
			break;
		case 9:
			strcpy(dev_type, "T2M");
			break;
		case 10:
			strcpy(dev_type, "GOMA");
			break;
		case 11:
			strcpy(dev_type, "MBUS");
			break;
		case 12:
			strcpy(dev_type, "T2V");
			break;
		case 13:
			strcpy(dev_type, "T2WF");
			break;
		case 14:
			strcpy(dev_type, "T2W");
			break;
		case 16:
			strcpy(dev_type, "GOTH");
			break;
		case 20:
			strcpy(dev_type, "CC");
			break;
		case 21:
			strcpy(dev_type, "T3ABC");
			break;
		case 22:
			strcpy(dev_type, "TCC");
			break;
	}

	return dev_type;
}

char *decode_sensor_data(uint8_t *pkt, uint32_t dev_type)
{
	int16_t data = 0;
	float dd  = 0;

	data = (pkt[11]  << 8) | pkt[12];

	switch(dev_type) {
		case 0:
		case 2:
		case 4:
			// Temperature
			dd = (float)(data / 10.0);
			ftoa(dev_data, dd, 1);
			sprintf(dev_data, "%s", dev_data);
			break;
		case 1:
		case 3:
			// Pressure
			dd = (float)(data / 100.0);
			ftoa(dev_data, dd, 2);
			sprintf(dev_data, "%s", dev_data);
			break;
		case 5:
			// ET-Pump
			dd = data;
			sprintf(dev_data, "0x%X", data);
			break;
		case 7:
			// water level (pressure)
			dd = (float)(data / 100.0);
			ftoa(dev_data, dd, 2);
			sprintf(dev_data, "%sM", dev_data);
			break;
		case 8:
			// Humi&Temp Sensor
			//dd = (float)(data / 10.0);
			//ftoa(dev_data, dd, 0);
			//sprintf(dev_data, "%s %d", dev_data, (int8_t)(pkt[20]));
			sprintf(dev_data, "%d %d", (int)(data/10.0+0.5), (int8_t)(pkt[20]));
			break;
		case 9:
			// Moving Sensor
			sprintf(dev_data, "%dMM", data);
			break;
		case 12:
			// Vibration Sensor
			sprintf(dev_data, "%d", data);
			break;
		case 13:
			// Float & Temp Sensor
			dd = (float)(data / 10.0);
			ftoa(dev_data, dd, 1);
			sprintf(dev_data, "%s", dev_data);
			break;
		case 14:
			// Water Leak Sensor
			dd = (float)(data / 10.0);
			ftoa(dev_data, dd, 1);
			sprintf(dev_data, "%s", dev_data);
			break;
		case 16:
			// GOTh with oled
			sprintf(dev_data, "%d %d", (int)(data/10.0+0.5), (int8_t)(pkt[20]));
			break;
		case 20:
			// Internal Temprature of ECC
			dd = (float)(data / 10.0);
			ftoa(dev_data, dd, 1);
			sprintf(dev_data, "%s", dev_data);
			break;
		case 21:
			// Internal Temprature of ABC Sensor
			dd = (float)(data / 10.0);
			ftoa(dev_data, dd, 1);
			sprintf(dev_data, "%s", dev_data);
			break;

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

#endif

#ifdef ENABLE_OLED
void show_frame(int l, int mode, bool alarm)
{
	u8g2.setPowerSave(0);

	u8g2.firstPage();

	do {
		u8g2.setFont(u8g2_font_freedoomr10_tu);

		for (int i=0; i < 8; i++) {
			u8g2.setCursor(0, 15+i*16);
			u8g2.print(frame_buf[i]);
		}

		if (l != 7) {
			// l == 7 is last 2 line, used by rssi

			if (MODE_CC2 == mode || MODE_RAW == mode || MODE_MAC == mode) {

				// Bell icon. notice the message tagged for testing
				//u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
				//u8g2.drawGlyph(120, 12+16*l*2, 65);
				u8g2.setCursor(124, 25+17*l);
				u8g2.print('.');

			} else if (MODE_ALL == mode || MODE_KEY == mode) {
				// cycle icon. notice the message trigged by magnet
				int ic = 64;

				if (alarm == false) {
					u8g2.setFont(u8g2_font_open_iconic_app_1x_t);

					u8g2.drawGlyph(120, 12+16*l, ic);
				} else {

					// show alarm icon
					if (dev_id[3] == '0' && (dev_id[4] == '1' || dev_id[4] == '3'
						|| dev_id[4] == '7' || dev_id[4] == '8')) {
						// Pressure & Level, Humidity digital sensor, showing lightning icon
						u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
						ic = 67;

						u8g2.drawGlyph(116, 15+16*l, ic);

					} else if (dev_id[3] == '1' && dev_id[4] == '3') {
						// Water Leak sensor, showing water drop icon
						u8g2.setFont(u8g2_font_open_iconic_thing_1x_t);
						ic = 72;

						u8g2.drawGlyph(118, 12+16*l, ic);

					} else if (dev_id[3] == '1' && dev_id[4] == '2') {
						// Vibration sensor, showing pulse icon
						u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);

						ic = 70;

						u8g2.drawGlyph(116, 15+16*l, ic);
					}

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
		u8g2.drawXBM(1, 52, logo_width, logo_height, logo_xbm);
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
			u8g2.setCursor(12, 64);
			u8g2.print(" T2 - ALL ");

			u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
			u8g2.drawGlyph(112, 61, 81);
		} else if (MODE_CC2 == mode) {
			// notice the message tagged for testing
			u8g2.setFont(u8g2_font_freedoomr10_tu);	// choose a suitable font
			u8g2.setCursor(12, 64);
			u8g2.print(" CC - 2.0 ");

			u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
			u8g2.drawGlyph(112, 61, 65);
		} else if (MODE_MAC == mode) {
			// notice the message tagged for testing
			u8g2.setFont(u8g2_font_freedoomr10_tu);	// choose a suitable font
			u8g2.setCursor(12, 64);
			u8g2.print(" CC - CMD ");

			u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
			u8g2.drawGlyph(112, 61, 65);
		} else if (MODE_KEY == mode) {
			// cycle icon. notice the message trigged by magnet
			u8g2.setFont(u8g2_font_freedoomr10_mu);	// choose a suitable font
			u8g2.setCursor(12, 64);
			u8g2.print(" T2 - KEY ");

			u8g2.setFont(u8g2_font_open_iconic_app_1x_t);
			u8g2.drawGlyph(112, 61, 64);
		} else if (MODE_RAW == mode) {
			// Lora icon. notice rx all raw message
			u8g2.setFont(u8g2_font_freedoomr10_mu);	// choose a suitable font
			u8g2.setCursor(12, 64);
			u8g2.print(" T2 - RAW ");

			u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
			u8g2.drawGlyph(112, 61, 81);
		}
	} while (u8g2.nextPage());
}

void show_low_bat()
{
	u8g2.setPowerSave(0);

	u8g2.firstPage();

	do {
		u8g2.setFont(u8g2_font_freedoomr10_mu);	// choose a suitable font
		u8g2.setCursor(16, 64);
		u8g2.print(" LOW BATTERY ");
	} while (u8g2.nextPage());
}

void show_rssi_err()
{
	sprintf(frame_buf[6], "RSSI:%3d ERR: %1d %1d", sx1272.getRSSI(), rx_err_cnt, rx_hung_cnt);
	//sprintf(frame_buf[7], "MDST: %02X  HUN: %1d", sx1272.get_modem_stat(), rx_hung_cnt);
	sprintf(frame_buf[7], "MDST: %02X RX:%d", sx1272.get_modem_stat(), rx_cnt);
	show_frame(7, omode, false);
}
#endif

void key_irq_handler()
{
	key_time = seconds();

	omode++;
	omode %= 5;
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
	int i, pos = 0;
	uint16_t hh = 0, sum = 0;

	pos = plen - 6;
	sum = p[pos] << 8 | p[pos+1];

	for (i = 0; i < pos; i++) {
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
#ifdef ENABLE_RX_INTERRUPT
	int8_t e = 0;

	sx1272.getRSSIpacket();
	e = sx1272.get_pkt_v0();

	if (!e) {
		uint8_t plen = sx1272._payloadlength;
		uint8_t *p = sx1272.packet_received.data;

		rx_cnt++;

		if (plen > PKT_LEN) plen = PKT_LEN;

		if (omode == MODE_RAW ||
			(omode == MODE_KEY && is_our_pkt(p, plen) && (p[15]&0x0F) >= 3 && (p[15]&0x0F) <= 5) ||
			((omode == MODE_CC2 || omode == MODE_ALL || omode == MODE_MAC) && is_our_pkt(p, plen))) {

			push_pkt(&g_cbuf, p, sx1272._RSSIpacket, plen);

		}

	} else {

		rx_err_cnt++;
	}

	//INFOLN("%d", e);

	//sx1272.rx_v0();
#endif
}

void reset_dev_sys()
{
	power_off_dev();
	delay(2300);			/* delay 2.3s */
	NVIC_SystemReset();
}

void setup()
{
	int e;

#if 0
	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = false;
	wInit.em3Run = false;
	wInit.perSel = wdogPeriod_2k;	/* 2k 1kHz periods should give 2 seconds */
#endif

#ifdef DEBUG
	Serial.setRouteLoc(1);
	Serial.begin(115200);
#endif

	INFOLN("%s", "Init start...");

	// Key connected to D0
	pinMode(KEY_PIN, INPUT);
	attachInterrupt(KEY_PIN, key_irq_handler, FALLING);

#ifdef ENABLE_RX_INTERRUPT
	// RF RX Interrupt pin
	pinMode(RX_INT_PIN, INPUT);
	attachInterrupt(RX_INT_PIN, rx_irq_handler, RISING);
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

	INFOLN("%s", "Init OK......");
	radio_setup();

#ifdef ENABLE_RX_INTERRUPT
	sx1272.init_rx_int();
	sx1272.rx_v0();
#endif

	/* Start watchdog */
	//WDOG_Init(&wInit);

	crypto_init();
}

void deep_sleep()
{

#ifdef ENABLE_OLED
	u8g2.setPowerSave(1);
#endif

	// dev power off
	power_off_dev();
	digitalWrite(SX1272_RST, LOW);

	spi_end();

	// reset the mode
	omode = MODE_ALL;

	EMU_EnterEM2(true);

	setup();
}

int get_1st_rssi(uint8_t *pkt)
{
	uint16_t vbat = pkt[13] << 8 | pkt[14];

	uint16_t ui16 = vbat / 10 * 10;

	if (pkt[17] >= 1) {

		return (vbat - ui16);

	} else {
		return -1;
	}
}

int get_temp_low1(uint8_t *pkt)
{
	int16_t data = (pkt[11]  << 8) | pkt[12];

	int ret = data -  10 * (data / 10);

	return (ret >= 0 ? ret : -ret);
}

void loop(void)
{
	int e = 1;
	static int c = 0;

	//WDOG_Feed();

#ifdef ENABLE_RX_INTERRUPT
	status_counter++;

	int rssi = sx1272.getRSSI();
	if (rssi <= -155 ||
		rssi == 0 ||
		(sx1272.get_modem_stat() & 0xB0)) {

		rx_hung_cnt++;
	}

	if (rx_hung_cnt >= 1) show_rssi_err();

	if (rx_err_cnt > RX_ERR_THRESHOLD || rx_hung_cnt >= 2) {

		show_rssi_err();

		INFO_S("%s", "Resetting lora module\n");

		if (rx_hung_cnt >= 1) {

			power_off_dev();
			delay(800);
			power_on_dev();

			u8g2.begin();
		}

		sx1272.reset();
		radio_setup();

		sx1272.init_rx_int();
		sx1272.rx_v0();

		rx_err_cnt = 0;
		rx_hung_cnt = 0;
	}
#endif

	if (omode != old_omode) {
#ifdef ENABLE_OLED
		// show mode and clean buffer
		show_mode(omode);

		for (int i = 0; i < 8; i++) {
			frame_buf[i][0] = '\0';
		}
#endif
		old_omode = omode;
	}

	if (digitalRead(KEY_PIN) == 0) {

		if (seconds() > (key_time + KEY_LONG_PRESS_TIME)) {

			key_time = 0;
			INFOLN("%s", "Key long time pressed, enter deep sleep...");

			deep_sleep();
		}

	} else {
		key_time = seconds();
	}

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

#else
	{
		struct pkt d;

		noInterrupts();
		int ret = get_pkt(&g_cbuf, &d);
		interrupts();

		if (ret != 0) {
			show_rssi_err();
			return;
		}

		uint8_t *p = d.data;
		uint8_t p_len = d.plen;

#endif /* ENABLE_RX_INTERRUPT */

#ifdef DEBUG_HEX_PKT
		int a = 0, b = 0;

		for (; a < p_len; a++, b++) {

			if ((uint8_t) p[a] < 16)
				INFO_S("%s", "0");

			INFO_HEX("%X", (uint8_t) p[a]);
			INFO_S("%s", " ");
		}

		INFO_S("%d", "/");
		INFO("%d", d.rssi);
		INFO_S("%s", "/");
		INFOLN("%d", p_len);
#endif

		if (p_len <= 11) return;

		status_counter = 0;

		memset(p+p_len, 0, MAX_PAYLOAD-p_len);

		decode_devid(p);

		uint32_t dev_t = get_dev_type(p);

#ifndef ENABLE_RX_INTERRUPT
		if (strcmp(dev_id, "") == 0) {
			// lora module unexpected error
			sx1272.reset();
		#if DEBUG > 1
			INFO_S("%s", "Resetting lora module\n");
		#endif
			radio_setup();
		}
#endif

		if (MODE_ALL == omode) {
			// show all message
			#ifdef DEBUG
			sprintf(cmd, "%s/U/%s/%s/%s/c/%d/v/%d/rssi/%d",
				dev_id,
				decode_vbat(p),
				decode_sensor_type(dev_t),
				decode_sensor_data(p, dev_t),
				decode_cmd(p),
				decode_ver(p),
				d.rssi);
			#endif

			#ifdef ENABLE_OLED
				#if 0
				sprintf(frame_buf[c % 8], "%s %4d",
					dev_id,
					d.rssi);

				show_frame(c % 8, omode, false);
				#endif
				int fi = (c % 3) * 2;
				//int fi = (c % 4) * 2;

				sprintf(frame_buf[fi], "%s %4d",
					dev_id,
					d.rssi);

				decode_vbat(p);
				decode_sensor_type(dev_t);
				decode_sensor_data(p, dev_t);

				if (dev_t == 8 || dev_t == 16) {

					sprintf(frame_buf[fi+1], "%s %s %s",
						dev_type,
						dev_data,
						dev_vbat
						);
				} else {
					sprintf(frame_buf[fi+1], " %s %s %s",
						dev_type,
						dev_data,
						dev_vbat
						);
				}

				show_frame(fi, omode, false);
				c++;
			#endif

				INFOLN("%s", cmd);
		} else if (MODE_CC2 == omode) {
			// only show the cc2.0 message
			if (p[2] == 0x34 || (p[2] == 0x33 && p_len == 36)) {

			#ifdef DEBUG
				sprintf(cmd, "%s/U/%s/%s/%s/ccid/%s/rssi/%d",
					decode_devid(p),
					decode_vbat(p),
					decode_sensor_type(dev_t),
					decode_sensor_data(p, dev_t),
					decode_ccid(p),
					d.rssi);
			#endif

			#ifdef ENABLE_OLED
				int fi = (c % 3) * 2;
				//int fi = (c % 4) * 2;

				int rssi_1st = get_1st_rssi(p);

				if (rssi_1st != -1) {
					sprintf(frame_buf[fi], "%s %1d %4d",
						dev_id,
						rssi_1st,
						d.rssi);
				} else {
					sprintf(frame_buf[fi], "%s  %4d",
						dev_id,
						d.rssi);
				}

				//if (strncmp(dev_id+3, "20", 2) == 0 && (p[11] & 0x80)) {

				if ((p[27] & 0x10) && p[17] >= 1 && p[17] <= 5) {

					sprintf(frame_buf[fi + 1], "%d %2d %2d %2d %2d %2d",
						check_pkt_mic(p, p_len),
						p[18],
						p[19],
						p[24],
						p[25],
						p[26]);

				} else {
					sprintf(frame_buf[fi + 1], "%d %s %s",
						check_pkt_mic(p, p_len),
						decode_vbat(p),
						decode_sensor_data(p, dev_t));
				}

				show_frame(fi, omode, false);
				c++;
			#endif
				INFOLN("%s", cmd);
			}
		} else if (MODE_MAC == omode) {

			if (dev_id[3] == '2' && dev_id[4] == '0' && p_len == 36) {

			#ifdef DEBUG
				sprintf(cmd, "%s/%02X/%d/%d/ccid/%s/rssi/%d",
					decode_devid(p),
					p[11],	/* mac cmd */
					p[12],	/* cad_on */
					decode_cc2_epoch(p),
					decode_ccid(p),
					d.rssi);
			#endif

			#ifdef ENABLE_OLED
				int fi = (c % 4) * 2;

				sprintf(frame_buf[fi], "%s %1d %4d",
					dev_id,
					p[17],
					d.rssi);

				int xep = decode_cc2_epoch(p);
				if (1600155579 == xep) {
					sprintf(frame_buf[fi + 1], "%1d %02X  %s  %02X  %d",
						check_pkt_mic(p, p_len),
						p[11],
						"ON",
						p[27],
						get_temp_low1(p));

				} else {
					sprintf(frame_buf[fi + 1], "%1d %02X %d %02X",
						check_pkt_mic(p, p_len),
						p[11],
						xep,
						p[27]);
				}
				show_frame(fi, omode, false);
				c++;
			#endif
				INFOLN("%s", cmd);
			}

		} else if (MODE_KEY == omode) {
			// only show trigged message

#ifndef ENABLE_RX_INTERRUPT
			if (p[0] != 0x47 || p[1] != 0x4F) {
				return;
			}
#endif

			if ((p[2] == 0x33 || p[2] == 0x34) && (p[15] == 0x03 || p[15] == 0x04 || p[15] == 0x05)) {

			#ifdef DEBUG
				sprintf(cmd, "%s/U/%s/%s/%s/rssi/%d",
					dev_id,
					decode_vbat(p),
					decode_sensor_type(dev_t),
					decode_sensor_data(p, dev_t),
					d.rssi);
			#endif

			#ifdef ENABLE_OLED
				int fi = (c % 3) * 2;
				sprintf(frame_buf[fi], "%s %4d",
					dev_id,
					d.rssi);

				if (dev_t == 8 || dev_t == 16) {

					sprintf(frame_buf[fi + 1], "%s %s %s",
						decode_sensor_type(dev_t),
						decode_sensor_data(p, dev_t),
						decode_vbat(p)
						);
				} else {
					sprintf(frame_buf[fi + 1], " %s %s %s",
						decode_sensor_type(dev_t),
						decode_sensor_data(p, dev_t),
						decode_vbat(p)
						);
				}

				show_frame(fi, omode, p[15] & 0x04);
				c++;
			#endif

				INFOLN("%s", cmd);
			}
		} else if (MODE_RAW == omode) {

			// only show raw message, <= 32bytes
			#ifdef ENABLE_OLED
				int fi = (c % 3) * 2;
				sprintf(frame_buf[fi], "%02X%02X%02X%02X%02X%02X%02X%02X%02X",
					p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]);

				sprintf(frame_buf[fi + 1], "%02X%02X %d %d %s%4d",
					p[9], p[10], p_len, check_crc(p, p_len), decode_vbat(p), d.rssi);

				show_frame(fi, omode, false);
				c++;
			#endif
		}
	}
}
