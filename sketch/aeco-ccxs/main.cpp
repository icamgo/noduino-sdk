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

#include "softi2c.h"

#include "rtcdriver.h"
#include "tx_ctrl.h"
#include "circ_buf.h"

#define ENABLE_TX5					1

#define CC_OPEN_WIN				150
//#define CC_CLOSE_WIN			1290
#define CC_CLOSE_WIN			6
#define CC_RPT_PERIOD			45
#define CC_HUNG_PERIOD 			(CC_RPT_PERIOD + 2)
#define CCTX_OFF_RPT_PERIOD		10

#if 0
#define	DEBUG						1
//#define DEBUG_TX					1
//#define DEBUG_RSSI					1
//#define DEBUG_DEVID					1
//#define DEBUG_HEX_PKT				1
#endif

//#define ENABLE_FLASH				1
#define ENABLE_CRYPTO				1
#define ENABLE_CAD					1
#define ENCODE_CCID_LOW1			1

#define ENABLE_ENG_MODE				1

#define	FW_VER						"V4.1"

#define LOW_BAT_THRESHOLD			3.1
#define RX_ERR_THRESHOLD			15

/* Timer used for bringing the system back to EM0. */
RTCDRV_TimerID_t xTimerForWakeUp;
static uint32_t check_period = 54;	/* 30s */

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

#ifdef ENCODE_FULL_CCID
#define MAC_FIRST_CCID			0x86
#define MAC_LATEST_CCID			0x87
#else
#define MAC_ENGMODE_ON			0x86
#define MAC_ENGMODE_OFF			0x87
#endif

#define MAC_RESET_CC			0x88

#define MAC_TX5_OFF				0x89
#define MAC_TX5_ON				0x8A

#ifdef ENABLE_FLASH
#define MAC_TX5_OFF_SAVE		0x8B
#define MAC_TX5_ON_SAVE			0x8C
#define MAC_CAD_OFF_SAVE		0x8D
#define MAC_CAD_ON_SAVE			0x8E
#endif

#define	PAYLOAD_LEN					30		/* 30+2+4 = 36B */
//#define	PAYLOAD_LEN					26		/* 26+2+4 = 32B */
//#define PAYLOAD_LEN					18		/* 18+2+4 = 24B */
#define OLED_DELAY_TIME				55		/* oled is on about 55s */

uint8_t rpt_pkt[PAYLOAD_LEN+6] __attribute__((aligned(4))) = { 0x47, 0x4F, 0x33 };

struct circ_buf g_cbuf __attribute__((aligned(4)));

struct ctrl_fifo g_cfifo __attribute__((aligned(4)));


#ifdef ENABLE_CRYPTO
#include "crypto.h"
#endif

#ifdef EFM32HG110F64
//#define ENABLE_OLED					1
//#define ENABLE_SH1106				1
//#define ENABLE_SSD1306			1
#endif

////////////////////////////////////////////////////////////
#ifdef EFM32GG230F512
/* EFM32GG */
//#define	PWR_CTRL_PIN			5		/* PIN06_PA05_D5 */
//#define	KEY_PIN					41		/* PIN36_PD08_D41 */
//#define	RX_INT_PIN				12		/* PIN16_PB08_D12 */

#define	PWR_CTRL_PIN			4		/* PIN05_PA04_D4 */
#define	KEY_PIN					54		/* PIN53_PF04_D54 */
#define	RX_INT_PIN				8		/* PIN18_PA09_D8 */

#define OLED_PWR3_PIN			55		/* PIN54_PF5_D55 */
#define OLED_PWR12_PIN			9		/* PIN19_PA10_D9 */
#define OLED_I2C_SDA			13		/* PIN21_PB11_D13 */
#define OLED_I2C_SCL			14		/* PIN22_PB12_D14 */

#else
/* EFM32ZG & EFM32HG */
#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define	KEY_PIN					0		/* PIN01_PA00_D0 */
#define	RX_INT_PIN				3		/* PIN8_PB11_D3 */
#endif


#ifdef CONFIG_V0
#define DEST_ADDR				1

bool cad_on __attribute__((aligned(4))) = true;
#define	CAD_TX_TIME					900			// 900ms
#define	NOCAD_TX_TIME				220			// 220ms
int tx_time = CAD_TX_TIME;

#define TXRX_CH					CH_01_472
#endif

#ifdef ENABLE_OLED && DEBUG
char cmd[128];
#endif

int oled_on_time = 0;
uint8_t rx_err_cnt __attribute__((aligned(4))) = 0;
uint8_t rx_hung_cnt __attribute__((aligned(4))) = 0;

uint32_t rx_cnt = 0;
uint32_t tx_cnt = 0;
uint32_t old_tx_cnt = 0;
uint32_t tx_cnt_1min = 0;

uint32_t old_rx_cnt = 0;
uint32_t cnt_1min = 0;

bool tx_on __attribute__((aligned(4))) = true;

#ifdef ENCODE_FULL_CCID
bool first_ccid __attribute__((aligned(4))) = false;
#endif

#ifdef ENABLE_ENG_MODE
/*
 * If full ccid is enable, do not support to
 * turn on/off the eng mode via mac cmd
*/
bool eng_mode_on __attribute__((aligned(4))) = true;
#else
bool eng_mode_on __attribute__((aligned(4))) = false;
#endif


#ifdef ENABLE_TX5
bool tx5_on __attribute__((aligned(4))) = true;
#else
bool tx5_on __attribute__((aligned(4))) = false;
#endif


#ifdef ENABLE_FLASH
#ifdef EFM32HG110F64
#define CFGDATA_BASE     (0x0000FC00UL) /* config data page base address: 64K - 1K */
#elif EFM32ZG110F32
#define CFGDATA_BASE     (0x00007C00UL) /* config data page base address: 32K - 1K */
#elif EFM32GG230F512
#define CFGDATA_BASE     (0x0000FC00UL) /* config data page base address: 64K - 1K */
#endif

uint32_t *cfg_addr = ((uint32_t *) CFGDATA_BASE);

typedef struct cfg_data {
	uint32_t init_flag;
	uint32_t epoch;
	uint32_t tx_count;
	uint32_t tx5_on;
	uint32_t cad_on;
	uint32_t tx_on;
} cfg_data_t;

cfg_data_t g_cfg __attribute__((aligned(4)));

void flash_init()
{
	uint32_t *p = (uint32_t *)&g_cfg;

	uint32_t flag = *cfg_addr;

	if (0x55aa == flag) {
		/*
		 * cfg in flash is used
		 * need to init the g_cfg
		*/
		memcpy(p, cfg_addr, sizeof(g_cfg));

		tx5_on = 0x1 & (g_cfg.tx5_on);
		cad_on = 0x1 & (g_cfg.cad_on);
	}
}

void flash_update()
{
	if (0x55aa == g_cfg.init_flag) {
		MSC_Init();
		MSC_ErasePage(cfg_addr);

		MSC_WriteWord(cfg_addr, &g_cfg, sizeof(g_cfg));
		MSC_Deinit();
	}
}
#endif


/*
 * Output Mode:
 *
 *   0x0: rx all messages
 *   0x1: show the raw message
*/
#define MODE_NUM		4

#define	MODE_RAW		0
#define	MODE_DECODE		1
#define MODE_STATIS		2
#define MODE_VER		3

int omode = MODE_STATIS;

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

#ifdef EFM32GG230F512
/*
 * PIN09_PC0: I2C0_SDA#4
 * PIN10_PC1: I2C0_SCL#4
*/

#define SH1107_RESET				2		/* PIN03_PA02_D2 */

U8G2_SH1107_SEEED_128X128_1_HW_I2C u8g2(U8G2_R0, SH1107_RESET);

#else

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
#endif // ENABLE_SH1106

#endif // ENABLE_OLED

float cur_vbat __attribute__((aligned(4))) = 0.0;
bool vbat_low __attribute__((aligned(4))) = false;
int cnt_vbat_low __attribute__((aligned(4))) = 0;
int cnt_vbat_ok __attribute__((aligned(4))) = 0;

static bool need_sleep __attribute__((aligned(4))) = false;
uint32_t cnt_sleep __attribute__((aligned(4))) = 0;

bool is_my_did(uint8_t *p);

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

	sx1272.init_rx_int();
	sx1272.rx_v0();
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

		if (MODE_DECODE == mode) {
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

#ifdef EFM32HG110F64
void change_omode()
{
	omode++;
	omode %= MODE_NUM;

#ifdef ENABLE_OLED
	oled_on = true;
	oled_on_time = seconds() + OLED_DELAY_TIME;
#endif

	INFO("omode: ");
	INFOLN(omode);
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
		#ifdef DEBUG_DEVID
		INFOLN("ivd0");
		#endif
		return false;
	}

	uint64_t devid = 0UL;
	uint8_t *pd = (uint8_t *) &devid;

	for(int i = 0; i < 8; i++) {
		pd[7-i] = p[3+i];
	}

	if (99999999999ULL == devid) {
		#ifdef DEBUG_DEVID
		INFOLN("999 ok");
		#endif
		return true;
	}

	if (devid > 99999999999ULL) {
		#ifdef DEBUG_DEVID
		INFOLN("ivd3");
		#endif
		return false;
	}

	uint32_t temp = devid / 10000;
	uint32_t tt = (uint32_t)(temp / 100);
	uint32_t wk_yr = (uint32_t)(temp - tt * 100);

	if (wk_yr > 52) {

		#ifdef DEBUG_DEVID
		INFOLN("ivd1");
		#endif

		return false;
	}

    temp = devid / 100000000;
    tt = (uint32_t)(temp / 100);
	wk_yr = (uint32_t)(temp - tt * 100);

	if (wk_yr > 30 || wk_yr < 18) {

		#ifdef DEBUG_DEVID
		INFOLN("ivd2");
		#endif
		return false;
	}

	// check dev_type
	temp = devid / 1000000;
	tt = (uint32_t)(temp / 100);
	uint32_t type = (uint32_t)(temp - tt * 100);

	/* only re-tx 02 & 03 & 13 */
	if (type != 2 && type != 3 && type != 13) {
		return false;
	}

	#ifdef DEBUG_DEVID
	INFOLN("did ok");
	#endif
	return true;
}

bool is_our_pkt(uint8_t *p, int len)
{

	/*
	 * 31: 17
	 * 32: 19
	 * 33: 20
	*/
	if (len < 23) return false;

	if (p[0] != 0x47 || p[1] != 0x4F) {

		// invalide pkt header
		return false;
	}

	// check the devid quickly in interrupt contex
	if (p[3] != 0 || p[4] != 0 || p[5] != 0 || p[6] > 0x17) {

		// invalid devid. max_id = 0x17 ff ff ff ff (1_030.79.21.5103)
		return false;
	}

	if (p[2] < 0x32 || p[2] > 0x36) {

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

uint64_t get_devid();

#ifdef ENCODE_FULL_CCID
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
#else
uint32_t get_ccid_low2(uint8_t *p)
{
	uint8_t ret = 0;

	if (1 <= p[17]) {

		ret = p[18];
	}

	return ret;
}

uint32_t get_ccid_low1(uint8_t *p)
{
	uint8_t ret = 0;

	if (1 <= p[17]) {

		ret = p[18];
	}

	return (ret - 10 * (ret / 10));
}
#endif

#ifndef ENCODE_FULL_CCID
uint8_t get_myid_low2()
{
	uint64_t devid = get_devid();

	uint64_t tt = (uint64_t)(devid / 100) * 100;

	uint8_t ret = (uint8_t)(devid - tt);

	return ret;
}
#endif

int map_rssi_val(int si)
{
	if (si <= -125) return 0;
	else if (si > -125 && si <= -120) return 1;
	else if (si > -120 && si <= -115) return 2;
	else if (si > -115 && si <= -110) return 3;
	else if (si > -110 && si <= -105) return 4;
	else if (si > -105 && si <= -100) return 5;
	else if (si > -100 && si <= -90) return 6;
	else if (si > -90 && si <= -80) return 7;
	else if (si > -80 && si <= -70) return 8;
	else if (si > -70) return 9;
}

void encode_temp_vbat(uint8_t *pkt, int rssi)
{
	float dd, vb;

	decode_devid(pkt);

	if ('2' == dev_id[3] && '0' == dev_id[4]) {
		// do not encode the cc rpt pkt
		return;
	}

	// encode the vbat
	uint16_t vbat = pkt[13] << 8 | pkt[14];

#ifdef ENCODE_CCID_LOW1
	vb = (float)(vbat / 1000.0 + (pkt[17] == 1 ? 1 : 0) * 0.005);
	uint16_t ui16 = vb * 100;

	ui16 *= 10;
	//ui16 += (uint16_t)get_ccid_low1(pkt);
	ui16 += (uint16_t)map_rssi_val(rssi);
#else
	vb = (float)(vbat / 1000.0 + (pkt[17] == 1 ? 1 : 0) * 0.05);
	uint16_t ui16 = vb * 10;

	ui16 *= 100;

	ui16 += (uint16_t)get_ccid_low2(pkt);
#endif

	uint8_t *pb = (uint8_t *) &ui16;
	pkt[13] = pb[1]; pkt[14] = pb[0];
	///////////////////

#if 0
	// encode the temp
	int16_t data = (pkt[11]  << 8) | pkt[12];

	if ('0' == dev_id[3] && ('0' == dev_id[4] || '2' == dev_id[4] || '8' == dev_id[4])) {
		// 0.1
		dd = (float)(data / 10.0);

		if (dd >= 0 && dd < 130) {

			data = (int16_t)(dd + (pkt[17] == 1 ? 1 : 0) * 0.5);
			data *= 10;

		} else {

			return;
		}

	} else if ('0' == dev_id[3] && ('1' == dev_id[4] || '3' == dev_id[4])) {
		// 0.01
		dd = (float)(data / 100.0);

		if (dd >= 1 && dd < 18) {

			data = (dd + (pkt[17] == 1 ? 1 : 0) * 0.05) * 10;
			data *= 10;

		} else {

			return;
		}

	} else if ('1' == dev_id[3] && '3' == dev_id[4]) {
		// 0.1
		dd = (float)(data / 10.0);

		if (dd > 1 && dd <= 100) {

			data = (int16_t)(dd + (pkt[17] == 1 ? 1 : 0) * 0.5);
			data *= 10;

		} else {

			return;
		}

	} else {

		return ;
	}

#if 0
	if (pkt[17] < 6) {
		data += pkt[17];
	} else {
		data += 9;
	}
#else
	data += map_rssi_val(rssi);
#endif

	pb = (uint8_t *) &data;
	pkt[11] = pb[1];
	pkt[12] = pb[0];
#endif
}

bool process_pkt(uint8_t *p, int *len, int rssi)
{
	//p[0] = 0x22;

	// p[15] is the cmd type

	if (true == is_my_did(p)) {
		/* No need to cc my own pkt */
		return true;
	}

	if (p[2] == 0x33 || p[2] == 0x32) {

		//crc ok, len=36 mic ok, extend the 0x33

		if (*len < 36) {

			/* move the frame no. to p[28:29] */
			//p[28] = p[16];
			//p[29] = p[17];
			p[28] = p[*len-8];
			p[29] = p[*len-7];

			if (0x32 == p[2]) {
				/*
				 * plen is 23
				 * p[15:16] is the fno.
				 * p[17:18] is the crc
				 * p[19:22] RFU
				*/
				p[2] = 0x33;
				p[15] = 0;

				/* p[17:18, 19:22] = 0 */
				memset(p+16, 0, *len-16);

				/* p[20:22] */
				//p[*len-3] = 0; p[*len-2] = 0; p[*len-1] = 0;

			} else {
				/*
				 * p[16]: fctrl
				 * p[17]: cc relayed counter
				 * p[18:19]: Fopts
				*/
				memset(p+16, 0, 4);
			}

			//p[20:23]: extend data

			// p[24:26]: low 3B of cc-devid
			// p[27]: MType
			//p[24] = 0; p[25] = 0; p[26] = 0; p[27] = 0;
			memset(p+24, 0, 4);

			// p[28:29]: frame no.

			// p[30:31] as the crc
			// p[32:35] as the reserved crc

			*len = 36;
		}
	}

	if (0x33 == p[2] && (*len == 36)) {

		/*
		 * 1. mark the cc flag
		 * 2. increment the cc cnt
		 * 3. fill the ccid
		 * 4. encode the did and rssi of the d
		*/
			if (p[27] & 0x10) {
				// new cc relayed pkt

				if (true == tx5_on) {

					if (p[17] >= 5) {

						// cc relayed times
						return false;

					} else {
						p[17] += 1;
					}
				}

			} else {
				// device pkt
				p[27] |= 0x10;		// mark as cc-relayed
				p[17] = 1;			// increment the cc-relayed-cnt

			#ifndef ENCODE_FULL_CCID
				p[18] = get_myid_low2();
			#endif
			}

			//uint8_t mtype = p[27] & 0xE0;
			//if (0x20 != mtype && 0x60 != mtype) {
			// pkt is not the down pkt

		#ifdef ENCODE_FULL_CCID
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
		#else
			if (true == tx5_on) {

				if (1 == p[17]) {

					p[18] = get_myid_low2();

				} else if (2 == p[17]) {

					p[19] = get_myid_low2();

				} else if (3 == p[17]) {

					p[24] = get_myid_low2();

				} else if (4 == p[17]) {

					p[25] = get_myid_low2();

				} else if (5 == p[17]) {

					p[26] = get_myid_low2();
				}
			}
		#endif

			if ((p[27] & 0x10) && eng_mode_on && (1 == p[17])) {
				// cc2.0 relayed & it's the first cc2.0 rx-pkt
				encode_temp_vbat(p, rssi);
			}

	} else if (0x34 == p[2]) {
		/*
		 * 1. mark the cc flag
		 * 2. increment the cc cnt
		 * 3. fill the ccid
		 * 4. encode the did and rssi of the d
		*/

		/* PAYLOAD_LEN = (*len-6) */
		if (p[*len-6-3] & 0x10) {
			// new cc relayed pkt

			if (true == tx5_on) {

				if (p[*len-6-5] >= 5) {

					// cc relayed times
					return false;

				} else {
					p[*len-6-5] += 1;
				}
			}

		} else {
			// device pkt
			p[*len-6-3] |= 0x10;		// mark as cc-relayed
			p[*len-6-5] = 1;			// increment the cc-relayed-cnt

		#ifndef ENCODE_FULL_CCID
			p[*len-6-6] = get_myid_low2();
		#endif
		}

		#ifdef ENCODE_FULL_CCID
		if ((1 == p[*len-6-5])
			|| (false == first_ccid && p[*len-6-5] > 1)) {

			uint64_t devid = get_devid();
			uint8_t *pd = (uint8_t *) &devid;

			p[*len-6-6] = pd[0];
			p[*len-6-7] = pd[1];
			p[*len-6-8] = pd[2];

			p[*len-6-9] = pd[3];
			p[*len-6-10] = pd[4];
		}
		//}
		#else
		if (true == tx5_on) {

			/* 1 ~ 5 */
			int tc = p[*len-6-5];

			if (tc >=1 && tc <=5) {
				p[*len-6-5-tc] = get_myid_low2();
			}
		}
		#endif

		if ((p[*len-6-3] & 0x10) && eng_mode_on && (1 == p[*len-6-5])) {
			// cc2.0 relayed & it's the first cc2.0 rx-pkt
			encode_temp_vbat(p, rssi);
		}
	}

	if (check_ctrl_fno(&g_cfifo, p, *len) == true) {

	#ifdef ENABLE_CRYPTO
		uint8_t mtype = p[27] & 0xE8;

		if (0x33 == p[2] && 36 == *len && (0x28 == mtype || 0x68 == mtype)) {

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

bool is_route_ok(uint8_t *p)
{
	uint8_t id = get_myid_low2();

	switch (p[17]) {
		case 4:
			if (p[25] == id) return false;
		case 3:
			if (p[24] == id) return false;
		case 2:
			if (p[19] == id) return false;
		case 1:
			if (p[18] == id) return false;
	}

	return true;
}

bool is_cced(uint8_t *p, int len)
{
	if (p[2] == 0x33 && len == 36) {

		if (p[27] & 0x10) {

			// new cc relayed pkt
			return false;

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

inline void set_mac_cmd(uint8_t cmd)
{
	mac_cmd = cmd;
	tx_cause = MAC_TX;
	need_push_mac = 0x55;

	tx_on = false;
}

inline void set_the_epoch(uint8_t *ep)
{
	uint32_t sec = seconds();
	uint8_t *st_p = (uint8_t *)&sec;
	st_p[0] = ep[3];
	st_p[1] = ep[2];
	st_p[2] = ep[1];
	st_p[3] = ep[0];

	reset_ctrl_ts(&g_cfifo, sec);
}

void process_mac_cmds(uint8_t *p, int len)
{
	uint8_t cmd = p[11];
	uint8_t vcmd = p[12];

	//uint8_t fctrl = p[16];
	// fopts: p[18:19], 20..23

	if ((~cmd & 0xFF) != vcmd) {
		// invalid cmd
		INFOLN(cmd);
		return;
	}

	uint32_t sec = 0;
	uint8_t *pd = (uint8_t *) &sec;

	pd[0] = p[23];
	pd[1] = p[22];
	pd[2] = p[21];
	pd[3] = p[20];

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
			set_mac_cmd(cmd);
			tx_on = false;
			//set_the_epoch(p+20);
			break;
		case MAC_CCTX_ON:
			set_mac_cmd(cmd);
			tx_on = true;
			//set_the_epoch(p+20);
			break;
		case MAC_SET_EPOCH:
			set_the_epoch(p+20);
			set_mac_cmd(cmd);
			break;
		case MAC_GET_CMD:
			set_mac_cmd(cmd);
			break;
		#ifdef ENABLE_CAD
		case MAC_CAD_OFF:
			set_mac_cmd(cmd);
			cad_on = false;
			tx_time = NOCAD_TX_TIME;
			break;
		case MAC_CAD_ON:
			set_mac_cmd(cmd);
			cad_on = true;
			tx_time = CAD_TX_TIME;
			break;
		#endif
		#ifdef ENCODE_FULL_CCID
		case MAC_FIRST_CCID:
			set_mac_cmd(cmd);
			first_ccid = true;
			break;
		case MAC_LATEST_CCID:
			set_mac_cmd(cmd);
			first_ccid = false;
			break;
		#else
		case MAC_ENGMODE_ON:
			set_mac_cmd(cmd);
			eng_mode_on = true;
			break;
		case MAC_ENGMODE_OFF:
			set_mac_cmd(cmd);
			eng_mode_on = false;
			break;
		#endif
		case MAC_RESET_CC:
			set_mac_cmd(cmd);
			break;
		case MAC_TX5_OFF:
			set_mac_cmd(cmd);
			tx5_on = false;
			break;
		case MAC_TX5_ON:
			set_mac_cmd(cmd);
			tx5_on = true;
			break;
		#ifdef ENABLE_FLASH
		case MAC_TX5_OFF_SAVE:
			set_mac_cmd(cmd);
			tx5_on = false;
			g_cfg.tx5_on = tx5_on;
			g_cfg.cad_on = cad_on;
			g_cfg.init_flag = 0x55aa;
			flash_update();
			break;
		case MAC_TX5_ON_SAVE:
			set_mac_cmd(cmd);
			tx5_on = true;
			g_cfg.tx5_on = tx5_on;
			g_cfg.cad_on = cad_on;
			g_cfg.init_flag = 0x55aa;
			flash_update();
			break;
		case MAC_CAD_OFF_SAVE:
			set_mac_cmd(cmd);
			cad_on = false;
			g_cfg.cad_on = cad_on;
			g_cfg.tx5_on = tx5_on;
			g_cfg.init_flag = 0x55aa;
			flash_update();
			break;
		case MAC_CAD_ON_SAVE:
			set_mac_cmd(cmd);
			cad_on = true;
			g_cfg.cad_on = cad_on;
			g_cfg.tx5_on = tx5_on;
			g_cfg.init_flag = 0x55aa;
			flash_update();
			break;
		#endif
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

bool is_did_for_me(uint8_t *p)
{
	// check the dev_id

	uint64_t pkt_did = 0UL;
	uint8_t *pd = (uint8_t *) &pkt_did;

	for(int i = 0; i < 8; i++) {
		pd[7-i] = p[3+i];
	}

	// 0x174876E7FF	= 999.99.99.9999
	if (get_devid() == pkt_did || 99999999999ULL == pkt_did) {

		return true;

	} else {
		return false;
	}
}

void rx_irq_handler()
{
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);

	int8_t e = 0;

	sx1272.getRSSIpacket();

	/* 0: ok; 1: crc error; 2: >max_len */
	e = sx1272.get_pkt_v0();

	if (!e) {

		uint8_t plen = sx1272._payloadlength;
		uint8_t *p = sx1272.packet_received.data;

		rx_cnt++;

		if (plen > PKT_LEN) return;

		if ((true == is_our_pkt(p, plen))
			&& (false == is_my_did(p))
			&& (true == is_cced(p, plen))
			&& (false == is_pkt_in_ctrl(&g_cfifo, p, plen, seconds()))) {

			// need to push into the tx queue buffer
			push_pkt(&g_cbuf, p, sx1272._RSSIpacket, plen);
		}

	} else {

		rx_err_cnt++;
	}

	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

int tx_pkt(uint8_t *p, int len)
{
	//radio_setup();
	bool need_cadon = true;
	int e = 0;

	if (false == is_my_did(p)) {

		/* pkt is not my rpt pkt */
		if (false == is_our_did(p) || p[2] < 0x33) {

			return 5;
		}
	}

	if (KEY_TX == p[15]) {

		if(is_my_did(p)) {
			// Turn off cad if pkt is my key_tx
			need_cadon = false;
		}
	}

	if (cad_on && need_cadon) {

		sx1272.CarrierSense();
		e = sx1272.sendPacketTimeout(DEST_ADDR, p, len, CAD_TX_TIME);

	} else {

		e = sx1272.sendPacketTimeout(DEST_ADDR, p, len, NOCAD_TX_TIME);
	}

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

#ifdef ENCODE_FULL_CCID
	/*
	 * x_bit0: tx_on
	 * x_bit1: cad_on
	 * x_bit2: first_ccid
	*/
	uint8_t xbit = first_ccid << 2 | cad_on << 1 | tx_on;
#else
	uint8_t xbit = tx5_on << 2 | cad_on << 1 | tx_on;
#endif

	return ret >= 0 ? (ret + xbit) : (ret - xbit);
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
	pkt[11] = mac_cmd; pkt[12] = 0;

	ui16 = get_encode_vbat();
	p = (uint8_t *) &ui16;
	pkt[13] = p[1]; pkt[14] = p[0];

	pkt[15] = tx_cause;

	uint32_t sec = seconds();
	uint8_t *ep = (uint8_t *)&sec;
	pkt[20] = ep[3];
	pkt[21] = ep[2];
	pkt[22] = ep[1];
	pkt[23] = ep[0];

	// pkt[27] = 0;		/* data up pkt, no crypto */
#ifdef ENCODE_FULL_CCID
	pkt[27] = (uint8_t)(first_ccid << 2 | cad_on << 1 | tx_on);
#else
	pkt[27] = (uint8_t)(eng_mode_on << 2 | cad_on << 1 | tx_on);
#endif

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

	uint32_t sec = seconds();
	uint8_t *ep = (uint8_t *)&sec;
	pkt[20] = ep[3];
	pkt[21] = ep[2];
	pkt[22] = ep[1];
	pkt[23] = ep[0];

	// pkt[27] = 0;		/* data up pkt, no crypto */
#ifdef ENCODE_FULL_CCID
	pkt[27] = (uint8_t)(first_ccid << 2 | cad_on << 1 | tx_on);
#else
	pkt[27] = (uint8_t)(eng_mode_on << 2 | cad_on << 1 | tx_on);
#endif

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

void reset_dev_sys()
{
	power_off_dev();
	i2c_delay(6000*I2C_1MS);			/* delay 6000ms */
	NVIC_SystemReset();
}

void wakeup_check(RTCDRV_TimerID_t id, void *user)
{
	/* reset the watchdog */
	WDOG_Feed();

	RTCDRV_StopTimer(xTimerForWakeUp);

	INFOLN("wakeup check");

	++cnt_sleep;

#if 0
	if (cnt_sleep % 130 == 0 && need_sleep == false) {
		/* work 130min */
		need_sleep = true;
	}
#endif

	if (cnt_sleep % CC_CLOSE_WIN == 0 && need_sleep == true) {

		need_sleep = false;
	}
}

extern "C" void seconds_callback()
{
	if (seconds() % 60 != 0) {
		return;
	}

	if (0 == digitalRead(KEY_PIN)) {
		return;
	}

	WDOG_Feed();

	INFOLN("xxxx");
	++cnt_1min;

	//if (cnt_1min % 5 == 0 && need_sleep == false) {
	if (cnt_1min % CC_OPEN_WIN == 0 && need_sleep == false) {
		/* work 150min */
		need_sleep = true;
	}

	if (need_sleep == true) {
		return;
	}

	//if (false == vbat_low) {

		//////////////////////////////////////////////////////
		/* check the rx_err in 1min */
		if (rx_err_cnt > (tx_cnt_1min + RX_ERR_THRESHOLD)) {

			need_reset_sx1272 = 0x55;

		}

		if (cnt_1min % CC_RPT_PERIOD == 0) {
			// 10min timer

			tx_cnt_1min = (tx_cnt - old_tx_cnt) / CC_RPT_PERIOD;
			old_tx_cnt = tx_cnt;

			// Timer report pkt
			tx_cause = TIMER_TX;
			need_push = 0x55;

		}

		if (MAC_CCTX_OFF == mac_cmd && (cnt_1min % CCTX_OFF_RPT_PERIOD == 0)) {
			// if cc-off, 2min report
			tx_cause = TIMER_TX;
			need_push_mac = 0x55;
		}

		if (cnt_1min % CC_HUNG_PERIOD == 0) {

			// 12min timer

			if (rx_cnt == old_rx_cnt) {
				// no rx pkt, reset the system
				//reset_dev_sys();
				rx_hung_cnt = 30;
				need_reset_sx1272 = 0x55;

			} else {

				old_rx_cnt = rx_cnt;
			}

		}

		#if 0
		if (cnt_1min % 1440 == 0) {
			// 24h
			reset_dev_sys();
		}
		#endif
	//}

#if 0
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

		cnt_vbat_ok = 0;

	} else {

		cnt_vbat_ok++;

		if (cnt_vbat_ok >= 10) {

			if (vbat_low) {
			#if 1
				/* Reset the system */
				reset_dev_sys();
			#else
				/*
				 * Recover from low vbat state
				 * Set the flag to reset the lora
				*/
				need_reset_sx1272 = 0x55;
				rx_hung_cnt = 4;
			#endif
			}

			vbat_low = false;
			cnt_vbat_ok = 0;
		}

		cnt_vbat_low = 0;
	}
#endif
}

#if defined(EFM32ZG110F32) || defined(EFM32GG230F512)
void key_report_status()
{
	#if 0
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
	#endif

	tx_cause = KEY_TX;
	need_push = 0x55;
}
#endif

void setup()
{
	int e;

	memset(&g_cbuf, 0, sizeof(struct circ_buf));
	memset(&g_cfifo, 0, sizeof(struct ctrl_fifo));

#ifdef ENABLE_CRYPTO
	CMU_ClockEnable(cmuClock_AES, true);
#endif

	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = wdogPeriod_64k;	/* 256k 1kHz periods should give 256 seconds */

	// Key connected to D0
	pinMode(KEY_PIN, INPUT);
#ifdef EFM32HG110F64
	attachInterrupt(KEY_PIN, change_omode, FALLING);
#else
	// EFM32ZG110F32 or EFM32GG230F512
	attachInterrupt(KEY_PIN, key_report_status, FALLING);
#endif

	// RF RX Interrupt pin
	pinMode(RX_INT_PIN, INPUT);
	attachInterrupt(RX_INT_PIN, rx_irq_handler, RISING);

	// dev power ctrl
	pinMode(PWR_CTRL_PIN, OUTPUT);

	power_on_dev();

#ifdef EFM32GG230F512
	// oled pwr ctrl
	pinMode(OLED_PWR3_PIN, OUTPUT);
	pinMode(OLED_PWR12_PIN, OUTPUT);
	pinMode(OLED_I2C_SCL, OUTPUT);
	pinMode(OLED_I2C_SDA, OUTPUT);

	digitalWrite(OLED_PWR3_PIN, LOW);
	digitalWrite(OLED_PWR12_PIN, LOW);
	digitalWrite(OLED_I2C_SCL, HIGH);
	digitalWrite(OLED_I2C_SDA, HIGH);
#endif


#ifdef ENABLE_OLED
	u8g2.begin();

	delay(1);
	show_logo();
	delay(800);

	if (adc.readVbat() < 2.92) {
		show_low_bat();
		delay(2700);
	}

	sprintf(frame_buf[0], " RX: %4d", rx_cnt);
	sprintf(frame_buf[1], " TX: %4d", tx_cnt);
	show_frame(0, omode, false);
#endif

#ifdef DEBUG
	Serial.setRouteLoc(1);
	Serial.begin(115200);
#endif

	/* Start watchdog */
	WDOG_Init(&wInit);

	/* reset epoch */
	update_seconds(160015579);
	reset_ctrl_ts(&g_cfifo, seconds());

	/* Initialize RTC timer. */
	RTCDRV_Init();
	RTCDRV_AllocateTimer(&xTimerForWakeUp);
	//RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypePeriodic, check_period * 1000, period_check_status, NULL);
	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypeOneshot, check_period * 1000, wakeup_check, NULL);

	radio_setup();

#ifdef ENABLE_OLED
	oled_on = true;
	oled_on_time = seconds() + OLED_DELAY_TIME;
#endif

	need_push = 0x55;
	tx_cause = RESET_TX;

#ifdef ENABLE_FLASH
	flash_init();
#endif

	INFOLN("Start setup");
}

void deep_sleep()
{

#ifdef ENABLE_OLED
	u8g2.setPowerSave(1);
#endif
	sx1272.setSleepMode();
	digitalWrite(SX1272_RST, LOW);

#ifdef USE_SOFTSPI
	spi_end();
#endif
	digitalWrite(10, LOW);
	digitalWrite(11, LOW);
	digitalWrite(12, LOW);
	digitalWrite(13, LOW);

	// dev power off
	power_off_dev();

	// reset the mode
	omode = MODE_DECODE;
}

void cc_worker();

void loop(void)
{
	if (need_sleep == true || 0 == digitalRead(KEY_PIN)) {
		/* storage mode */

		//INFOLN("deep sleep..");

		WDOG_Feed();

		deep_sleep();

		RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypeOneshot, check_period * 1000, wakeup_check, NULL);

		EMU_EnterEM2(true);

		delay(30000);
	}

	if (need_sleep == false && 1 == digitalRead(KEY_PIN)) {
		cc_worker();
	}
}

struct pkt d;

void cc_worker()
{
	int e = 1;
	static int c = 0;

	int rssi = sx1272.getRSSI();
	if (rssi <= -155 ||
		rssi == 0 ||
		(sx1272.get_modem_stat() & 0xB0)) {

		rx_hung_cnt++;
	}

	WDOG_Feed();

	if (0x55 == need_reset_sx1272 || rx_hung_cnt >= 1) {

		INFO_S("Reset lora module\n");
		INFO("rx_err_cnt = ");
		INFO(rx_err_cnt);
		INFO(" rx_hung_cnt = ");
		INFOLN(rx_hung_cnt);

		if (rx_hung_cnt >= 1) {
			power_off_dev();
			delay(1000);
			power_on_dev();
		#ifdef ENABLE_OLED
			u8g2.begin();
		#endif
		}

		sx1272.reset();
		radio_setup();			/* reset and setup */

		need_reset_sx1272 = 0;
		rx_err_cnt = 0;
		rx_hung_cnt = 0;
	}

#ifdef ENABLE_OLED
	if (seconds() > oled_on_time) {

		oled_on_time = 0;
		oled_on = false;

		u8g2.setPowerSave(1);

		omode = MODE_DECODE;
	}
#endif

	noInterrupts();
	int ret = get_pkt(&g_cbuf, &d);
	interrupts();

	if (ret != 0) {
		// there is no pkt
	#ifdef DEBUG_RSSI
		INFO("No pkt, rssi = ");
		INFOLN(sx1272.getRSSI());
	#endif
		goto process_rpt;
	}

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
	INFO(d.rssi);
	INFO_S("/");
	INFOLN(p_len);
#endif

#ifdef ENABLE_CRYPTO
	// (p[2] == 0x33 && 36 == len)
	if (false == check_pkt_mic(p, p_len)) {
		/* check the mic of pkt */
		goto process_rpt;
	}
#endif

	// filter the pkt
	if (0x33 == p[2] && 36 == p_len) {

		uint8_t mtype = p[27] & 0xE8;
		if (0x28 == mtype || 0x68 == mtype) {

			// down & crypto pkt
	#ifdef ENABLE_CRYPTO
			payload_decrypt(p, p_len, ae33kk);
	#endif

			if (is_did_for_me(p)) {
				process_mac_cmds(p, p_len);
			}

			if (true == is_my_did(p)) {
				/*
				 * is only for me or send by me
				 * no need to re-tx or show
				*/
				goto process_rpt;
			}
		}
	}

#if 0
	/* Is need to re-tx ? */
	if (is_cc_ok(p, p_len) == false) {

		/* cc flag and count is not ok */
		goto process_rpt;
	}

	if (true == is_pkt_in_ctrl(&g_cfifo, p, p_len, seconds())) {
		/* cc ctrl is not passed */
		goto process_rpt;
	}
#endif

	if (false == is_our_did(p) && false == is_my_did(p)) {
		/* filter the did */
		goto process_rpt;
	}

#ifdef ENABLE_OLED
	if (oled_on == true) {

		//memset(p+p_len, 0, MAX_PAYLOAD-p_len);
		decode_devid(p);

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

		if (MODE_RAW == omode) {

			// only show raw message, <= 32bytes
			sprintf(frame_buf[0], "%02X%02X%02X%02X%02X%02X%02X%02X%02X",
				p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]);

			sprintf(frame_buf[1], "%02X%02X %d %d %s%4d",
				p[9], p[10], p_len, check_crc(p, p_len), decode_vbat(p), d.rssi);

			show_frame(0, omode, false);

		} else if (MODE_STATIS == omode) {

			sprintf(frame_buf[0], " RX: %4d", rx_cnt);
			sprintf(frame_buf[1], " TX: %4d", tx_cnt);

			show_frame(0, omode, false);

		} else if (MODE_DECODE == omode) {

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

		} else if (MODE_VER == omode) {
			sprintf(frame_buf[0], " FW: %s", FW_VER);
			//sprintf(frame_buf[1], " EP: %d", seconds());

			uint64_to_str(get_devid());
			sprintf(frame_buf[1], " ID: %s", dev_id);

			show_frame(0, omode, false);
		}
	}
#endif

	if (tx_on) {

		if (process_pkt(p, &p_len, d.rssi) == true) {

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

process_rpt:

	if (0x55 == need_push) {

		set_temp_pkt();
	#ifdef ENABLE_CRYPTO
		set_pkt_mic(rpt_pkt, PAYLOAD_LEN+6);
	#endif

		if (tx_on) {
			noInterrupts();
			push_pkt(&g_cbuf, rpt_pkt, 0, PAYLOAD_LEN+6);
			interrupts();

			need_push = 0;
		} else {

		#ifdef ENABLE_CRYPTO
			set_pkt_mic(rpt_pkt, PAYLOAD_LEN+6);
		#endif

			e = tx_pkt(rpt_pkt, PAYLOAD_LEN+6);		/* 36B */

			if (0 == e) {
				need_push = 0;
			}

			sx1272.rx_v0();
		}
	}

	if (0x55 == need_push_mac) {

		set_mac_status_pkt();

	#ifdef ENABLE_CRYPTO
		set_pkt_mic(rpt_pkt, PAYLOAD_LEN+6);
	#endif

		if (tx_on) {
			noInterrupts();
			push_pkt(&g_cbuf, rpt_pkt, 0, PAYLOAD_LEN+6);
			interrupts();

			need_push_mac = 0;

			if (MAC_RESET_CC == mac_cmd) {
				NVIC_SystemReset();
			}

		} else {

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
