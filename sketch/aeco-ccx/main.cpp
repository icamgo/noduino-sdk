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

#include "rtcdriver.h"
#include "tx_ctrl.h"
#include "circ_buf.h"

#if 0
#define	DEBUG						1
#define DEBUG_TX					1
#define DEBUG_HEX_PKT				1
#endif

#define ENABLE_EXP_CC				1

#define ENABLE_CRYPTO				1
#define ENABLE_CAD					1

#define	FW_VER						"V2.2"

#define LOW_BAT_THRESHOLD			3.3

/* Timer used for bringing the system back to EM0. */
RTCDRV_TimerID_t xTimerForWakeUp;
static uint32_t check_period = 54;	/* 60s */

static uint32_t need_push = 0;
static uint32_t need_push_mac = 0;
static uint32_t need_reset_sx1272 = 0;
static uint32_t mac_cmd = 0;
static uint32_t mac_cmd_sec = 0;
static uint16_t tx_count __attribute__((aligned(4))) = 0;

#define	RESET_TX			0
#define	DELTA_TX			1
#define	TIMER_TX			2
#define	KEY_TX				3
#define	MAC_TX				10
static uint8_t tx_cause __attribute__((aligned(4))) = RESET_TX;

#define MAC_CCTX_OFF			0x80
#define MAC_CCTX_ON				0x81
#define MAC_SET_EPOCH			0x82
#define MAC_GET_CMD				0x83
#define MAC_CAD_OFF				0x84
#define MAC_CAD_ON				0x85
#define MAC_FIRST_CCID			0x86
#define MAC_LATEST_CCID			0x87
#define MAC_RESET_CC			0x88

#define	PAYLOAD_LEN					30		/* 30+2+4 = 36B */
//#define	PAYLOAD_LEN					26		/* 26+2+4 = 32B */
//#define PAYLOAD_LEN					18		/* 18+2+4 = 24B */

uint8_t rpt_pkt[PAYLOAD_LEN+6] __attribute__((aligned(4))) = { 0x47, 0x4F, 0x33 };

struct circ_buf g_cbuf __attribute__((aligned(4)));
#define OLED_DELAY_TIME				55		/* oled is on about 55s */

struct ctrl_fifo g_cfifo __attribute__((aligned(4)));

#ifdef ENABLE_CRYPTO
#include "crypto.h"
#endif

#ifdef EFM32HG110F64
#define ENABLE_OLED					1
#define ENABLE_SH1106				1
//#define ENABLE_SSD1306			1
#endif

////////////////////////////////////////////////////////////
#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define	KEY_PIN					0		/* PIN01_PA00_D0 */
#define	BEEP_PIN				10		/* PIN13_PD06_D10 */
#define	RX_INT_PIN				3		/* PIN8_PB11_D3 */


#ifdef CONFIG_V0
#define DEST_ADDR				1

bool cad_on __attribute__((aligned(4))) = true;
#define	CAD_TX_TIME					900			// 900ms
#define	NOCAD_TX_TIME				220			// 220ms
int tx_time = CAD_TX_TIME;

#define TXRX_CH					CH_01_472
#endif

#ifdef DEBUG
#define MAX_CMD_LENGTH			100
char cmd[MAX_CMD_LENGTH];
#endif

int oled_on_time = 0;
uint8_t rx_err_cnt __attribute__((aligned(4))) = 0;

uint32_t rx_cnt = 0;
uint32_t tx_cnt = 0;
uint32_t old_tx_cnt = 0;
uint32_t tx_cnt_1min = 0;

uint32_t old_rx_cnt = 0;
uint32_t cnt_1min = 0;

bool tx_on __attribute__((aligned(4))) = true;
bool first_ccid __attribute__((aligned(4))) = false;

/*
 * Output Mode:
 *
 *   0x0: rx all messages
 *   0x1: show the raw message
*/
#define MODE_NUM		5

#define	MODE_ALL		0
#define	MODE_RAW		1
#define	MODE_DECODE		2
#define MODE_STATIS		3
#define MODE_VER		4

int omode = MODE_STATIS;
int old_omode = MODE_STATIS;

bool oled_on __attribute__((aligned(4))) = true;

#ifdef DEBUG
#define INFO_S(param)			Serial.print(F(param))
#define INFO_HEX(param)			Serial.print(param,HEX)
#define INFO(param)				Serial.print(param)
#define INFOLN(param)			Serial.println(param)
#define FLUSHOUTPUT					Serial.flush();
#else
#define INFO_S(param)
#define INFO(param)
#define INFOLN(param)
#define FLUSHOUTPUT
#endif

#ifdef ENABLE_OLED
#include "U8g2lib.h"
#include "logo.h"

char frame_buf[2][24] __attribute__((aligned(4)));

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

float cur_vbat __attribute__((aligned(4))) = 0.0;
bool vbat_low __attribute__((aligned(4))) = false;
int cnt_vbat_low __attribute__((aligned(4))) = 0;
//__attribute__((aligned(4)))

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
	oled_on_time = seconds() + OLED_DELAY_TIME;

	INFO("omode: ");
	INFOLN(omode);
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

// check the devid not in interrupt contex
bool is_our_did(uint8_t *p)
{
	if (p[3] != 0 || p[4] != 0 || p[5] != 0 || p[6] > 0x17) {

		// invalid devid. max_id = 0x17 ff ff ff ff (1_030.79.21.5103)
		INFOLN("ivd0");
		return false;
	}

	uint64_t devid = 0UL;
	uint8_t *pd = (uint8_t *) &devid;

	for(int i = 0; i < 8; i++) {
		pd[7-i] = p[3+i];
	}

	if (99999999999ULL == devid) {
		INFOLN("999 ok");
		return true;
	}

	if (devid > 99999999999ULL) {
		INFOLN("ivd3");
		return false;
	}

	uint32_t temp = devid / 10000;
	uint32_t tt = (uint32_t)(temp / 100);
	uint32_t wk_yr = (uint32_t)(temp - tt * 100);

	if (wk_yr > 52) {

		INFOLN("ivd1");

		return false;
	}

    temp = devid / 100000000;
    tt = (uint32_t)(temp / 100);
	wk_yr = (uint32_t)(temp - tt * 100);

	if (wk_yr > 30 || wk_yr < 18) {

		INFOLN("ivd2");
		return false;
	}

	INFOLN("did ok");
	return true;
}

bool is_our_pkt(uint8_t *p, int len)
{

	if (len < 24) return false;

	if (p[0] != 0x47 || p[1] != 0x4F) {

		// invalide pkt header
		return false;
	}

	// check the devid quickly in interrupt contex
	if (p[3] != 0 || p[4] != 0 || p[5] != 0 || p[6] > 0x17) {

		// invalid devid. max_id = 0x17 ff ff ff ff (1_030.79.21.5103)
		return false;
	}

	if (p[2] < 0x33 || p[2] > 0x36) {

		// support only the 0x33/34/35/36 version
		return false;
	}

	if (check_crc(p, len) == false) {

		return false;
	}

#ifdef ENABLE_CRYPTO
	// (p[2] == 0x33 && 36 == len)
	return check_pkt_mic(p, len);
#endif

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

uint64_t get_devid();

#ifdef ENABLE_EXP_CC
uint32_t get_ccid_low2(uint8_t *pkt)
{
	uint64_t devid = 0UL;
	uint8_t *id_p = (uint8_t *)&devid;

	id_p[4] = pkt[18];
	id_p[3] = pkt[19];
	id_p[2] = pkt[24];
	id_p[1] = pkt[25];
	id_p[0] = pkt[26];

	uint64_t tt = (uint64_t)(devid / 100) * 100;
	uint32_t ret = (uint32_t)(devid - tt);

	return ret;
}

void encode_temp_vbat(uint8_t *pkt)
{
	float dd, vb;

	decode_devid(pkt);

	if ('2' == dev_id[3] && '0' == dev_id[4]) {
		// do not encode the cc rpt pkt
		return;
	}

	// encode the vbat
	uint16_t vbat = pkt[13] << 8 | pkt[14];

	vb = (float)(vbat / 1000.0 + 0.05);
	uint16_t ui16 = vb * 10;

	ui16 *= 100;
	ui16 += (uint16_t)get_ccid_low2(pkt);

	uint8_t *pb = (uint8_t *) &ui16;
	pkt[13] = pb[1]; pkt[14] = pb[0];
	///////////////////

	// encode the temp
	int16_t data = (pkt[11]  << 8) | pkt[12];

	if ('0' == dev_id[3] && ('2' == dev_id[4] || '8' == dev_id[4])) {
		// 0.1
		dd = (float)(data / 10.0);

		if (dd >= 0 && dd < 130) {

			data = (int16_t)(dd + 0.5);
			data *= 10;

		} else {

			return;
		}

	} else if ('0' == dev_id[3] && '3' == dev_id[4]) {
		// 0.01
		dd = (float)(data / 100.0);

		if (dd >= 0 && dd < 18) {

			data = (int16_t)(dd + 0.05);
			data *= 100;

		} else {

			return;
		}

	} else if ('1' == dev_id[3] && '3' == dev_id[4]) {
		// 0.1
		dd = (float)(data / 10.0);

		if (dd > 1 && dd <= 100) {

			data = (int16_t)(dd + 0.5);
			data *= 10;

		} else {

			return;
		}
	}

	if (pkt[17] < 6) {
		data += pkt[17];
	} else {
		data += 9;
	}

	pb = (uint8_t *) &data;
	pkt[11] = pb[1];
	pkt[12] = pb[0];

}
#endif

bool process_pkt(uint8_t *p, int *len)
{
	//p[0] = 0x22;

	// p[15] is the cmd type

	if (p[2] == 0x33) {

		//extend the 0x33

		if (*len == 24 || *len == 28 || *len == 32) {

			/* move the frame no. to p[28:29] */
			//p[28] = p[16];
			//p[29] = p[17];
			p[28] = p[*len-8];
			p[29] = p[*len-7];

			/*
			 * p[16]: fctrl
			 * p[17]: cc relayed counter
			 * p[18:19]: Fopts
			*/
			memset(p+16, 0, 4);

			//p[20:23]: extend data

			// p[24:26]: low 3B of cc-devid
			// p[27]: MType
			p[24] = 0; p[25] = 0; p[26] = 0; p[27] = 0;

			// p[28:29]: frame no.

			// p[30:31] as the crc
			// p[32:35] as the reserved crc

			*len = 36;
		}

		if (*len == 36) {

			if (p[27] & 0x10) {
				// new cc relayed pkt

				if (p[17] >= 5) {

					// cc relayed times
					return false;

				} else {
					p[17] += 1;
				}

			} else {
				// device pkt
				p[27] |= 0x10;		// mark as cc-relayed
				p[17] = 1;			// increment the cc-relayed-cnt
			}


			//uint8_t mtype = p[27] & 0xE0;
			//if (0x20 != mtype && 0x60 != mtype) {
			// pkt is not the down pkt

			if ((1 == p[17])
				|| (false == first_ccid && p[17] > 1)) {

				uint64_t devid = get_devid();
				uint8_t *pd = (uint8_t *) &devid;

				p[26] = pd[0];
				p[25] = pd[1];
				p[24] = pd[2];

				p[19] = pd[3];
				p[18] = pd[4];
			}
			//}

		#ifdef ENABLE_EXP_CC
			if (p[27] & 0x10) {
				// has ccid
				encode_temp_vbat(p);
			}
		#endif
		}

	}

	if (check_ctrl_fno(&g_cfifo, p, *len) == true) {

	#ifdef ENABLE_CRYPTO
		uint8_t mtype = p[27] & 0xE0;

		if (0x33 == p[2] && 36 == *len && (0x20 == mtype || 0x60 == mtype)) {

			// data down pkt (from gw/ctrl_client)
			payload_encrypt(p, *len, ae33kk);
		}
	#endif

		update_crc(p, *len);

		return true;

	} else {
		return false;
	}
}

bool is_cc_ok(uint8_t *p, int len)
{
	if (p[2] == 0x33 && len == 36) {

		if (p[27] & 0x10) {

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

uint64_t get_devid()
{
	uint64_t *p;

	p = (uint64_t *)0x0FE00008;

	return *p;
}

inline void turn_tx_off(uint8_t cmd)
{
	mac_cmd = cmd;
	tx_on = false;

	tx_cause = MAC_TX;
	need_push_mac = 0x55;
}

inline void turn_tx_on(uint8_t cmd)
{
	mac_cmd = cmd;
	tx_on = true;

	tx_cause = MAC_TX;
	need_push_mac = 0x55;
}

inline void set_the_epoch(uint8_t *ep)
{
	extern uint32_t secTicks;
	uint8_t *st_p = (uint8_t *)&secTicks;
	st_p[0] = ep[3];
	st_p[1] = ep[2];
	st_p[2] = ep[1];
	st_p[3] = ep[0];
}

void process_mac_cmds(uint8_t *p, int len)
{
	uint8_t cmd = p[11];
	uint8_t vcmd = p[12];

	//uint8_t fctrl = p[16];
	// fopts: p[18:19], 20..23

	uint64_t did = 0;
	uint8_t *pd = (uint8_t *) &did;

	if ((~cmd & 0xFF) != vcmd) {
		// invalid cmd
		INFOLN(cmd);
		return;
	}

	for(int i = 0; i < 8; i++) {
		pd[7-i] = p[3+i];
	}

	// check the dev_id
	// 0x174876E7FF	= 999.99.99.9999
	if (did == 99999999999ULL || did == get_devid()) {

		uint32_t sec = 0;
		pd = (uint8_t *) &sec;
		pd[0] = p[23]; pd[1] = p[22]; pd[2] = p[21]; pd[3] = p[20];

		/*
		 * check the sec-ts
		 *
		*/
		if (sec <= mac_cmd_sec) {
		//if (sec == mac_cmd_sec) {
			// invalide mac ctrl pkt
			return;
		} else {
			mac_cmd_sec = sec;
		}

		INFOLN("ok");
		switch(cmd) {
			case MAC_CCTX_OFF:
				turn_tx_off(cmd);
				set_the_epoch(p+18);
				break;
			case MAC_CCTX_ON:
				turn_tx_on(cmd);
				set_the_epoch(p+18);
				break;
			case MAC_SET_EPOCH:
				set_the_epoch(p+18);
				mac_cmd = cmd;
				tx_cause = MAC_TX;
				need_push_mac = 0x55;
				break;
			case MAC_GET_CMD:
				tx_cause = MAC_TX;
				need_push_mac = 0x55;
				break;
			#ifdef ENABLE_CAD
			case MAC_CAD_OFF:
				mac_cmd = cmd;
				cad_on = false;
				tx_time = NOCAD_TX_TIME;
				tx_cause = MAC_TX;
				need_push_mac = 0x55;
				break;
			case MAC_CAD_ON:
				mac_cmd = cmd;
				cad_on = true;
				tx_time = CAD_TX_TIME;
				tx_cause = MAC_TX;
				need_push_mac = 0x55;
				break;
			#endif
			case MAC_FIRST_CCID:
				mac_cmd = cmd;
				first_ccid = true;
				tx_cause = MAC_TX;
				need_push_mac = 0x55;
				break;
			case MAC_LATEST_CCID:
				mac_cmd = cmd;
				first_ccid = false;
				tx_cause = MAC_TX;
				need_push_mac = 0x55;
				break;
			case MAC_RESET_CC:
				mac_cmd = cmd;
				tx_cause = MAC_TX;
				need_push_mac = 0x55;
				break;
		}
	}

#if 0
	if (did == 99999999999ULL) {
		// need to cc this pkt
		memcpy(rpt_pkt, p, 32);
		need_push = 0x55;
	}
#endif
}

bool is_my_did(uint8_t *p)
{
	uint64_t pkt_did = 0UL;
	uint8_t *pd = (uint8_t *) &pkt_did;

	for(int i = 0; i < 8; i++) {
		pd[7-i] = p[3+i];
	}

	uint64_t devid = get_devid();

	if (pkt_did == devid)
		return true;
	else
		return false;
}

void rx_irq_handler()
{
	int8_t e = 0;

	sx1272.getRSSIpacket();

	/* 0: ok; 1: crc error; 2: >max_len */
	e = sx1272.get_pkt_v0();

	if (!e) {

		uint8_t plen = sx1272._payloadlength;
		uint8_t *p = sx1272.packet_received.data;

		if (plen > 44) return;

		rx_cnt++;

		if (is_our_pkt(p, plen) == true) {

			uint8_t mtype = p[27] & 0xE0;

			if (0x33 == p[2] && 36 == plen && (0x20 == mtype || 0x60 == mtype)) {

				// data down pkt (from gw/ctrl_client)

				if (p[27] & 0x08) {
			#ifdef ENABLE_CRYPTO
					payload_decrypt(p, plen, ae33kk);
			#endif
				}

				process_mac_cmds(p, plen);
			}

			if (is_cc_ok(p, plen) &&
				false == is_my_did(p) &&
				is_pkt_in_ctrl(&g_cfifo, p, plen, seconds()) == false) {

				// need to push into the tx queue buffer
				push_pkt(&g_cbuf, sx1272.packet_received.data, sx1272._RSSIpacket, plen);
			}
		}

	} else {

		rx_err_cnt++;

	}
}

int tx_pkt(uint8_t *p, int len)
{
	//radio_setup();

	if (false == is_our_did(p) || p[2] < 0x33) {

		return 5;
	}

#ifdef ENABLE_CAD
	if (cad_on) {
		sx1272.CarrierSense();
	}
#endif
	int e = sx1272.sendPacketTimeout(DEST_ADDR, p, len, tx_time);

	if (0 == e) {
		// send message succesful,
		#ifdef DEBUG_TX
		INFOLN("TX OK");
		#endif
	}

	return e;
}

int16_t get_encode_mcu_temp()
{
	float temp = 0.0;
	for(int i=0; i<3; i++) {
		temp += adc.temperatureCelsius();
	}
	temp /= 3.0;

	int16_t ret = (int16_t)(temp + 0.5) * 10;

	/*
	 * x_bit0: tx_on
	 * x_bit1: cad_on
	 * x_bit2: first_ccid
	*/
	uint8_t xbit = first_ccid << 2 | cad_on << 1 | tx_on;

	return ret + xbit;
}

uint16_t get_encode_vbat()
{
	/*
	 *  1: encode the tx_cnt/min
	 * 10: encode the tx_cnt/min
	*/
	float vbat = adc.readVbat();
	uint16_t ui16 = vbat * 10;

	ui16 *= 100;

	if (tx_cnt_1min > 99) {
		tx_cnt_1min = 99;
	}

	ui16 += (uint16_t)tx_cnt_1min;

	return ui16;
}

uint16_t get_crc(uint8_t *pp, int len)
{
	int i;
	uint16_t hh = 0;

	for (i = 0; i < len; i++) {
		hh += pp[i];
	}
	return hh;
}

void set_mac_status_pkt()
{
	uint8_t *pkt = rpt_pkt;
	int16_t ui16 = 0;

	memset(pkt, 0, 32);
	pkt[0] = 0x47; pkt[1] = 0x4F; pkt[2] = 0x33;

	uint64_t devid = get_devid();

	uint8_t *p = (uint8_t *) &devid;

	// set devid
	int i = 0;
	for(i = 0; i < 8; i++) {
		pkt[3+i] = p[7-i];
	}

	/*
	 * 0x80 00 = -3276.8
	 * 0x81 00 = -3251.2
	 * 0x82 00 = -3225.6
	*/
	pkt[11] = mac_cmd; pkt[12] = cad_on;

	ui16 = get_encode_vbat();
	p = (uint8_t *) &ui16;
	pkt[13] = p[1]; pkt[14] = p[0];

	pkt[15] = tx_cause;

	extern uint32_t secTicks;
	uint8_t *ep = (uint8_t *)&secTicks;
	pkt[20] = ep[3];
	pkt[21] = ep[2];
	pkt[22] = ep[1];
	pkt[23] = ep[0];

	// pkt[27] = 0;		/* data up pkt, no crypto */
	pkt[27] = (uint8_t)(first_ccid << 2 | cad_on << 1 | tx_on);

	p = (uint8_t *) &tx_count;
	pkt[PAYLOAD_LEN-2] = p[1]; pkt[PAYLOAD_LEN-1] = p[0];
	tx_count++;

	ui16 = get_crc(pkt, PAYLOAD_LEN);
	p = (uint8_t *) &ui16;
	pkt[PAYLOAD_LEN] = p[1]; pkt[PAYLOAD_LEN+1] = p[0];
}

void set_temp_pkt()
{
	uint8_t *pkt = rpt_pkt;

	memset(pkt, 0, 32);
	pkt[0] = 0x47; pkt[1] = 0x4F; pkt[2] = 0x33;

	uint64_t devid = get_devid();

	uint8_t *p = (uint8_t *) &devid;

	// set devid
	int i = 0;
	for(i = 0; i < 8; i++) {
		pkt[3+i] = p[7-i];
	}

	int16_t ui16 = get_encode_mcu_temp();
	p = (uint8_t *) &ui16;
	pkt[11] = p[1]; pkt[12] = p[0];

	ui16 = get_encode_vbat();
	p = (uint8_t *) &ui16;
	pkt[13] = p[1]; pkt[14] = p[0];

	// tx_cause = TIMER_TX
	pkt[15] = tx_cause;

	extern uint32_t secTicks;
	uint8_t *ep = (uint8_t *)&secTicks;
	pkt[20] = ep[3];
	pkt[21] = ep[2];
	pkt[22] = ep[1];
	pkt[23] = ep[0];

	// pkt[27] = 0;		/* data up pkt, no crypto */
	pkt[27] = (uint8_t)(first_ccid << 2 | cad_on << 1 | tx_on);

	p = (uint8_t *) &tx_count;
	pkt[PAYLOAD_LEN-2] = p[1]; pkt[PAYLOAD_LEN-1] = p[0];
	tx_count++;

	ui16 = get_crc(pkt, PAYLOAD_LEN);
	p = (uint8_t *) &ui16;
	pkt[PAYLOAD_LEN] = p[1]; pkt[PAYLOAD_LEN+1] = p[0];
}

float fetch_vbat()
{
	float vbat = 0;

	for (int i = 0; i < 3; i++) {
		vbat += adc.readVbat();
	}

	return vbat/3.0;
}

void period_check_status(RTCDRV_TimerID_t id, void *user)
{
	/* reset the watchdog */
	WDOG_Feed();

	if (false == vbat_low) {

		//////////////////////////////////////////////////////
		if (rx_err_cnt > 65) {

			need_reset_sx1272 = 0x55;
		}

		rx_err_cnt = 0;

		++cnt_1min;

		if (cnt_1min % 10 == 0) {
			// 10min timer

			tx_cnt_1min = (tx_cnt - old_tx_cnt) / 10;
			old_tx_cnt = tx_cnt;

			tx_cause = TIMER_TX;
			need_push = 0x55;
		}

		if (MAC_CCTX_OFF == mac_cmd && (cnt_1min % 2 == 0)) {
			// if cc-off, 2min report
			tx_cause = TIMER_TX;
			need_push_mac = 0x55;
		}

		if (cnt_1min % 60 == 0) {
			// 1h

			if (rx_cnt == old_rx_cnt) {
				// no rx pkt, wait the watchdog to reset
				//while(1);
				NVIC_SystemReset();

			} else {

				old_rx_cnt = rx_cnt;
			}
		}
	}

	////////////////////////////////////////////////////
	// check the low vbat
	cur_vbat = fetch_vbat();

	if (cur_vbat <= LOW_BAT_THRESHOLD) {

		cnt_vbat_low++;

		if (cnt_vbat_low >= 5) {
			// my battery is low

			vbat_low = true;
			cnt_vbat_low = 0;

			need_push = 0x55;
			tx_cause = DELTA_TX;
		}
	} else {

		if (vbat_low) {
			// need to reset the system
			//NVIC_SystemReset();
			need_reset_sx1272 = 0x55;
		}

		cnt_vbat_low = 0;
		vbat_low = false;
	}
}

#ifdef EFM32ZG110F32
void key_report_status()
{
	uint8_t *pkt = rpt_pkt;

	uint64_t devid = get_devid();

	uint8_t *p = (uint8_t *) &devid;

	// set devid
	int i = 0;
	for(i = 0; i < 8; i++) {
		pkt[3+i] = p[7-i];
	}

	int16_t ui16 = get_encode_mcu_temp();
	p = (uint8_t *) &ui16;
	pkt[11] = p[1]; pkt[12] = p[0];

	ui16 = get_encode_vbat();
	pkt[13] = p[1]; pkt[14] = p[0];

	// tx_cause = KEY_TX
	pkt[15] = KEY_TX;

	p = (uint8_t *) &tx_count;
	pkt[PAYLOAD_LEN-2] = p[1]; pkt[PAYLOAD_LEN-1] = p[0];
	tx_count++;

	ui16 = get_crc(pkt, PAYLOAD_LEN);
	p = (uint8_t *) &ui16;
	pkt[PAYLOAD_LEN] = p[1]; pkt[PAYLOAD_LEN+1] = p[0];

	tx_cause = KEY_TX;
	need_push = 0x55;
}
#endif

void setup()
{
	int e;

#ifdef ENABLE_CRYPTO
	CMU_ClockEnable(cmuClock_AES, true);
#endif

	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = wdogPeriod_64k;	/* 64k 1kHz periods should give 64 seconds */

	// Key connected to D0
	pinMode(KEY_PIN, INPUT);
#ifdef EFM32HG110F64
	attachInterrupt(KEY_PIN, change_omode, FALLING);
#endif

#ifdef EFM32ZG110F32
	attachInterrupt(KEY_PIN, key_report_status, FALLING);
#endif

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

	//show_mode(omode);
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

	/* Initialize RTC timer. */
	extern uint32_t secTicks;
	secTicks = 1600155579;

	RTCDRV_Init();
	RTCDRV_AllocateTimer(&xTimerForWakeUp);
	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypePeriodic, check_period * 1000, period_check_status, NULL);

	need_push = 0x55;
	tx_cause = RESET_TX;
}

void loop(void)
{
	int e = 1;
	static int c = 0;

	if (vbat_low) {

		// sleep to waitting for recharge the battery

		if (omode != old_omode) {

			show_low_bat();
			delay(2000);

			old_omode = omode;
		}

		sx1272.setSleepMode();

	#ifdef ENABLE_OLED
		u8g2.setPowerSave(1);
	#endif

		power_off_dev();
		digitalWrite(SX1272_RST, LOW);
		spi_end();

		EMU_EnterEM2(true);

	} else {

		if (0x55 == need_reset_sx1272) {

			power_on_dev();
			u8g2.begin();

			sx1272.reset();
			INFO_S("Resetting lora module\n");
			radio_setup();

			sx1272.init_rx_int();
			sx1272.rx_v0();

			need_reset_sx1272 = 0;
		}

		if (omode != old_omode) {
	#ifdef ENABLE_OLED
			// show mode and clean buffer
			//show_mode(omode);

			frame_buf[0][0] = '\0';
			frame_buf[1][0] = '\0';
	#endif
			old_omode = omode;
		}

		if (seconds() > oled_on_time) {

			oled_on_time = 0;
			oled_on = false;

	#ifdef ENABLE_OLED
			u8g2.setPowerSave(1);
	#endif
			omode = MODE_STATIS;
			old_omode = omode;
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
				INFO_S("0");

			INFO_HEX((uint8_t) p[a]);
			INFO_S(" ");
		}

		INFO_S("/");
		INFO(sx1272._RSSIpacket);
		INFO_S("/");
		INFOLN(p_len);
	#endif

		if (false == is_our_did(p)) return;

		if (oled_on == true) {

			//memset(p+p_len, 0, MAX_PAYLOAD-p_len);
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

					INFOLN(cmd);
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
			} else if (MODE_VER == omode) {
			#ifdef ENABLE_OLED
				sprintf(frame_buf[0], " FW: %s", FW_VER);
				//sprintf(frame_buf[1], " EP: %d", seconds());

				uint64_to_str(get_devid());
				sprintf(frame_buf[1], " ID: %s", dev_id);

				show_frame(0, omode, false);

			#endif
			}
		}

		if (tx_on) {

			if (process_pkt(p, &p_len) == true) {

			#ifdef ENABLE_CRYPTO
				set_pkt_mic(p, p_len);
			#endif

				e = tx_pkt(p, p_len);

				if (0 == e) {

					tx_cnt++;

				} else if (5 != e) {
					// valide pkt, other tx failed issue
					noInterrupts();
					push_pkt(&g_cbuf, p, d.rssi, p_len);
					interrupts();
				}

				sx1272.rx_v0();
			}
		}

		if (0x55 == need_push) {

			set_temp_pkt();

		#ifdef ENABLE_CRYPTO
			set_pkt_mic(rpt_pkt, PAYLOAD_LEN+6);
		#endif

			e = tx_pkt(rpt_pkt, PAYLOAD_LEN+6);		/* 36B */

			if (0 == e) {
				// tx successful
				need_push = 0;
			}

			sx1272.rx_v0();
		}

		if (0x55 == need_push_mac) {

			set_mac_status_pkt();

		#ifdef ENABLE_CRYPTO
			set_pkt_mic(rpt_pkt, PAYLOAD_LEN+6);
		#endif

			e = tx_pkt(rpt_pkt, PAYLOAD_LEN+6);		/* 36B */

			if (0 == e) {
				// tx successful
				need_push_mac = 0;

				if (MAC_RESET_CC == mac_cmd) {
					NVIC_SystemReset();
				}
			}

			sx1272.rx_v0();
		}
	}
}
