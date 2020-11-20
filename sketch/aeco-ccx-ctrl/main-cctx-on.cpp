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

#define	DEBUG					1
//#define DEBUG_HEX_PKT			1

#define ENABLE_CRYPTO				1

#ifdef ENABLE_CRYPTO
#include "crypto.h"
#endif

#define	PAYLOAD_LEN					30		/* 30+2+4 = 36B */
//#define	PAYLOAD_LEN					26		/* 26+2+4 = 32B */

#define ENABLE_OLED					1
#define ENABLE_SH1106				1
//#define ENABLE_SSD1306			1

#define ENABLE_CAD					1

#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define	KEY_PIN					0		/* PIN01_PA00_D0 */
#define	BEEP_PIN				10		/* PIN13_PD06_D10 */

#define SYNCWORD_DEFAULT		0x12
#define SYNCWORD_LORAWAN		0x34
#define SYNCWORD_ABC			0x55

#define DEST_ADDR				1

#ifdef CONFIG_V0

#define	RESET_TX			0
#define	DELTA_TX			1
#define	TIMER_TX			2
#define	KEY_TX				3
#define	EL_TX				4
#define	WL_TX				5
#define	CTRL_TX				10

#define RECEIVE_ALL
#define TXRX_CH				CH_01_472
#define	TX_TIME				1600			// 800ms
uint8_t loraMode = 12;

uint8_t pkt[PAYLOAD_LEN+6] = { 0x47, 0x4F, 0x33 };
uint8_t tx_cause = CTRL_TX;
uint16_t tx_count = 0;

uint32_t key_cnt = 0;

#else

#define TXRX_CH				CH_00_470		// 470.0MHz
#define	TX_TIME					200		// 200ms

// Default LoRa mode BW=125KHz, CR=4/5, SF=12
uint8_t loraMode = 11;
// Gateway address: 1
uint8_t loraAddr = 1;

#endif


#define MAC_CCTX_OFF			0x80
#define MAC_CCTX_ON				0x81
#define MAC_SET_EPOCH			0x82
#define MAC_GET_CMD				0x83
#define MAC_CAD_OFF				0x84
#define MAC_CAD_ON				0x85

#define MAC_ENGMODE_ON			0x86
#define MAC_ENGMODE_OFF			0x87

#define MAC_RESET_CC			0x88

#define MAC_TX5_OFF				0x89
#define MAC_TX5_ON				0x8A

#define MAC_TX5_OFF_SAVE		0x8B
#define MAC_TX5_ON_SAVE			0x8C

/*
 * TX CMD:
 *
 *   0x0: All CC Off
 *   0x1: All CC On
*/
#define	MODE_CC_OFF		0x0
#define	MODE_CC_ON		0x1

int txcmd = MODE_CC_ON;
int old_txcmd = MODE_CC_ON;

int key_time = 0;
bool key_pressed = false;

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
	//sx1272.setSyncWord(SYNCWORD_ABC);
#else
	sx1272.sx1278_qsetup(CH_00_470, 20);
	sx1272._nodeAddress = loraAddr;
#endif

#ifdef ENABLE_CAD
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
			vbat = pkt[13] << 8 | pkt[14];
			break;
	}

	ftoa(dev_vbat, (float)(vbat / 1000.0), 3);
	return dev_vbat;
}

char *decode_sensor_data(uint8_t *pkt)
{
	int16_t data = 0;
	float dd  = 0;

	data = (pkt[11]  << 8) | pkt[12];

	sprintf(dev_data, "%d", data);

	return dev_data;
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
#endif

#ifdef ENABLE_OLED
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
		if (MODE_CC_ON == mode) {
			u8g2.setFont(u8g2_font_freedoomr10_mu);
			u8g2.setCursor(12, 26);
			u8g2.print(" CC - ON ");

			u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
			u8g2.drawGlyph(112, 23, 81);
		} else if (MODE_CC_OFF == mode) {
			u8g2.setFont(u8g2_font_freedoomr10_mu);
			u8g2.setCursor(12, 26);
			u8g2.print(" CC - OFF");

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

void change_txcmd()
{
	key_cnt++;

#if 0
	if (key_cnt % 12 == 0) {

		if (MODE_CC_ON == txcmd) {

			txcmd = MODE_CC_OFF;

		} else if (MODE_CC_OFF == txcmd) {

			txcmd = MODE_CC_ON;
		}
	}

	INFO("%s", "txcmd: ");
	INFOLN("%d", txcmd);
#endif

	key_pressed = true;
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

void power_on_dev()
{
	digitalWrite(PWR_CTRL_PIN, HIGH);
}

void power_off_dev()
{
	digitalWrite(PWR_CTRL_PIN, LOW);
}

int send_cmd(uint8_t cmd)
{
	int e = 0;
	int16_t ui16 = 0;

	memset(pkt, 0, PAYLOAD_LEN+6);

	pkt[0] = 0x47; pkt[1] = 0x4F; pkt[2] = 0x33;

	uint64_t devid = 99999999999;					/* 0x17 48 76 E7 FF */

	uint8_t *p = (uint8_t *) &devid;

	// set devid
	int i = 0;
	for(i = 0; i < 8; i++) {
		pkt[3+i] = p[7-i];
	}

	pkt[11] = cmd; pkt[12] = ~cmd;

	float vbat = adc.readVbat();
	ui16 = vbat * 1000;
	pkt[13] = p[1]; pkt[14] = p[0];

	pkt[15] = tx_cause;

	// 18:19

	// 20:23
	uint32_t ep = seconds();
	p = (uint8_t *) &ep;
	pkt[20] = p[3]; pkt[21] = p[2]; pkt[22] = p[1]; pkt[23] = p[0];

	INFOLN("%d", ep);

	// pkt[24:27]

	/*
	 * Unconfirmed down data
	 * no-cc relayed
	 * Encrypt data
	 *
	 */
	pkt[27] = 0x28;

	// pkt[28:29]: frame no.
	p = (uint8_t *) &tx_count;
	pkt[PAYLOAD_LEN-2] = p[1]; pkt[PAYLOAD_LEN-1] = p[0];
	tx_count++;

	/////////////////////////////////////////////////////////
	/*
	 * 1. encrypt
	 * 2. crc
	 * 3. set mic
	*/
#ifdef ENABLE_CRYPTO
	payload_encrypt(pkt, PAYLOAD_LEN+6, ae33kk);
#endif

	p[15] = tx_cause;

	// pkt[30:31]: crc
	ui16 = get_crc(pkt, PAYLOAD_LEN);
	p = (uint8_t *) &ui16;
	pkt[PAYLOAD_LEN] = p[1]; pkt[PAYLOAD_LEN+1] = p[0];

#ifdef ENABLE_CRYPTO
	set_pkt_mic(pkt, PAYLOAD_LEN+6);
#endif
	/////////////////////////////////////////////////////////

#ifdef ENABLE_CAD
	sx1272.CarrierSense();
#endif

	// here we resend the received data to the next gateway
	e = sx1272.sendPacketTimeout(DEST_ADDR, pkt, PAYLOAD_LEN+6, TX_TIME);

	INFO_S("%s", "Packet re-sent, state ");
	INFOLN("%d", e);

	// set back the gateway address
	sx1272._nodeAddress = DEST_ADDR;

	return e;
}

void setup()
{
	int e;

#ifdef ENABLE_CRYPTO
	CMU_ClockEnable(cmuClock_AES, true);
#endif

	/* Initialize RTC timer. */
	extern uint32_t secTicks;
	secTicks = 1600155579;

	// Key connected to D0
	pinMode(KEY_PIN, INPUT);
	attachInterrupt(KEY_PIN, change_txcmd, FALLING);

	// beep
	pinMode(BEEP_PIN, OUTPUT);
	digitalWrite(BEEP_PIN, LOW);

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

	show_mode(txcmd);
#endif
	switch (txcmd) {
		case MODE_CC_OFF:
			send_cmd(MAC_CCTX_OFF);
			send_cmd(MAC_CCTX_OFF);
			send_cmd(MAC_CCTX_OFF);
			break;
		case MODE_CC_ON:
			send_cmd(MAC_RESET_CC);
			send_cmd(MAC_RESET_CC);
			send_cmd(MAC_RESET_CC);
			send_cmd(MAC_RESET_CC);
			send_cmd(MAC_RESET_CC);
			break;
	}

#ifdef DEBUG
	Serial.setRouteLoc(1);
	Serial.begin(115200);
#endif

	radio_setup();
}

void loop(void)
{
	int e = 1;
	static int c = 0;

	if (txcmd != old_txcmd || key_pressed) {
#ifdef ENABLE_OLED
		// show mode and clean buffer
		show_mode(txcmd);
#endif
		old_txcmd = txcmd;

		switch (txcmd) {
			case MODE_CC_OFF:
				send_cmd(MAC_CCTX_OFF);
				send_cmd(MAC_CCTX_OFF);
				send_cmd(MAC_CCTX_OFF);
				break;
			case MODE_CC_ON:
				send_cmd(MAC_RESET_CC);
				send_cmd(MAC_RESET_CC);
				send_cmd(MAC_RESET_CC);
				send_cmd(MAC_RESET_CC);
				send_cmd(MAC_RESET_CC);
				break;
		}

		key_pressed = false;

	}

	if (digitalRead(KEY_PIN) == 0) {
		// x 200ms
		key_time++;
	} else {
		key_time = 0;
	}

	if (key_time > 6) {

		key_time = 0;

		INFOLN("%s", "Key long time pressed, enter deep sleep...");
		delay(300);

#ifdef ENABLE_OLED
		u8g2.setPowerSave(1);
#endif
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

	delay(1000);
}
