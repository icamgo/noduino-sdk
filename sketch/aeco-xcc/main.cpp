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

#include <stdio.h>
#include <stdint.h>

#include "softi2c.h"
#include "pcf8563.h"

#include "sx126x.h"

#include "circ_buf.h"
#include "flash.h"

#define DEBUG					1
//#define ENABLE_CRYPTO			1


#ifdef ENABLE_CRYPTO
#include "crypto.h"
#endif
/*
 *
*/
#define SRC_TX_PERIOD	600
#define RPT_TX_PERIOD	600
#define RTC_PERIOD		30

#define TX_PWR			22			// dBm tx output power

uint32_t work_win __attribute__((aligned(4))) = 6;
uint32_t pair_win __attribute__((aligned(4))) = 30;

////////////////////////////////////////////////////////////
#define TXRX_CH			472500000	// Hz center frequency
#define	PAYLOAD_LEN		30			/* 30+2+4 = 36B */

#ifdef EFM32GG230F512
/* EFM32GG */
#define	PWR_CTRL_PIN			4		/* PIN05_PA04_D4 */
#define	KEY_PIN					54		/* PIN53_PF04_D54 */
#define	RF_INT_PIN				8		/* PIN18_PA09_D8 */
#else
/* EFM32ZG & EFM32HG */
#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define	KEY_PIN					0		/* PIN01_PA00_D0 */
#define	RF_INT_PIN				3		/* PIN8_PB11_D3 */
#define	RTC_INT_PIN				16		/* PIN21_PF02_D16 */

#define SDA_PIN					12		/* PIN23_PE12 */
#define SCL_PIN					13		/* PIN24_PE13 */
#endif

#define	RESET_TX			0
#define	DELTA_TX			1
#define	TIMER_TX			2
#define	KEY_TX				3
#define	MAC_TX				10


SX126x lora(2,		// Pin: SPI CS,PIN06-PB08-D2
	    9,				// Pin: RESET, PIN18-PC15-D9
	    5,				// PIN: Busy,  PIN11-PB14-D5, The RX pin
	    3				// Pin: DIO1,  PIN08-PB11-D3
    );

uint8_t rpt_pkt[PAYLOAD_LEN+6] __attribute__((aligned(4)));
struct circ_buf g_cbuf __attribute__((aligned(4)));

bool need_work __attribute__((aligned(4))) = true;
bool vbat_low __attribute__((aligned(4))) = false;
bool need_paired __attribute__((aligned(4))) = false;
bool rx_sync __attribute__((aligned(4))) = false;
bool tx_rpt __attribute__((aligned(4))) = false;
bool need_rpt __attribute__((aligned(4))) = false;

int32_t cnt_rtc_min __attribute__((aligned(4))) = 0;
uint32_t rtc_period __attribute__((aligned(4))) = RTC_PERIOD;

uint32_t start_ts __attribute__((aligned(4))) = 0;
uint32_t tx_cnt_1h __attribute__((aligned(4))) = 0;
uint32_t tx_cause = RESET_TX;

#ifdef DEBUG
#define INFO_S(param)			Serial.print(F(param))
#define INFOHEX(param)			Serial.print(param,HEX)
#define INFO(param)				Serial.print(param)
#define INFOLN(param)			Serial.println(param)
#define FLUSHOUTPUT				Serial.flush();
#else
#define INFO_S(param)
#define INFO(param)
#define INFOLN(param)
#define INFOHEX(param)
#define FLUSHOUTPUT
#endif

void radio_setup();
int tx_pkt(uint8_t *p, int len);
void set_cc_rpt();
void cc_worker();
void rx_worker();

inline uint64_t read_devid()
{
	uint64_t *p;

	p = (uint64_t *)0x0FE00008;

	return *p;
}

inline uint32_t get_prog_ts()
{
	uint32_t *p;

	p = (uint32_t *)0x0FE00004;

	return *p;
}

inline float fetch_vbat()
{
	float vbat = 0;

	for (int i = 0; i < 3; i++) {
		vbat += adc.readVbat();
	}

	return vbat/3.0;
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

uint64_t get_devid(uint8_t *p)
{
	uint64_t devid = 0UL;
	uint8_t *pd = (uint8_t *) &devid;

	for(int i = 0; i < 8; i++) {
		pd[7-i] = p[3+i];
	}
	return devid;
}

bool is_my_pkt(uint8_t *p, int len)
{
	/*
	 * 31: 17
	 * 32: 19
	 * 33: 20
	*/
	if (len < 36) return false;

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

	if (p[len-9] & 0x10) {
		// new cc relayed pkt
		return false;
	}

	uint32_t dtype = get_dev_type(p);

	if (dtype <= 28) {
		return false;
	}

	// pkt[27] = 0b100, set bit 2, HTimer rpt pkt
	if (0 == (p[len-9] & 0x4)) {
		// It's not the HTimer rpt pkt
		return false;
	}

#if 0
	if (check_crc(p, len) == false) {

		return false;
	}
#endif

	#ifdef ENABLE_CRYPTO
	if (check_pkt_mic(p, len) == 0) {
		return false;		
	}
	#endif

	return true;
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

void sync_rtc()
{
	rtc_period = RTC_PERIOD;
	cnt_rtc_min = 0;

	pcf8563_clear_timer();
	pcf8563_set_timer_s(rtc_period);

	rx_sync = true;
}

uint8_t pbuf[48];

void rx_irq_handler()
{
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);

	int len = 0, rssi = 0;

	WDOG_Feed();

	INFOLN("rx");

	len = lora.get_rx_pkt(pbuf, 48);
	rssi = lora.get_pkt_rssi();

	if (len > PKT_LEN || len < 0) goto rx_irq_out;

	if (is_my_pkt(pbuf, len) && need_work && pbuf[15] == TIMER_TX) {
		// if the devid is the white id
		// reset the timer
		sync_rtc();
		need_paired = false;

		// paired ok
		g_cfg.paired_did = get_devid(pbuf);
		g_cfg.paired_rx_ts = pcf8563_now();

		push_pkt(&g_cbuf, pbuf, rssi, len);
	}

rx_irq_out:
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

#ifdef DEBUG
void hex_pkt(uint8_t *p, int rssi, int plen)
{
	int a = 0;

	for (; a < plen; a++) {

		if ((uint8_t) p[a] < 16)
			Serial.print("0");

		Serial.print((uint8_t) p[a], HEX);
		Serial.print(" ");
	}

	Serial.print("/");
	Serial.print(rssi);
	Serial.print("/");
	Serial.print(plen);

	#ifndef ENABLE_CRYPTO
	Serial.println("/");
	#else
	Serial.print("/");
	Serial.println(check_pkt_mic(p, plen));
	#endif

}
#endif

void key_irq_handler()
{
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	/*
	 * 1. report white list id
	 * 2. waiting 30s? for the new white list id
	*/

	/* disable the rtc timer */
	pcf8563_clear_timer();

	/* need to report the paired id */
	tx_cause = KEY_TX;
	need_rpt = true;

	/* open the rx window */
	need_work = true;
	need_paired = true;
	rx_sync = false;
	start_ts = seconds();

	INFOLN("key");

	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

void rtc_irq_handler()
{
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);

	WDOG_Feed();

	cnt_rtc_min++;

	if (false == rx_sync) {
		/*
		 * rtc_period is: [0, 255], (255, 600)
		 *
		 * and rtc_period % 60 == 0 or != 0
		 *
		*/
		uint32_t pps = rtc_period % 60;

		if (rtc_period <= 255 || (rtc_period > 255 && (pps == 0))) {

			/* reset to normal rtc_period */
			rtc_period = RTC_PERIOD;

			/* open or close the every 30s rx win */
			pcf8563_clear_timer();

			/* re-open the rx window */
			need_work = true;
			need_paired = true;
			start_ts = seconds();

			goto rx_out;

		} else if ((rtc_period > 255) && (pps > 0)) {

			/* pps: (0, 59] */
			rtc_period = pps;

			/* waiting for next rtc irq */
			pcf8563_set_timer_s(rtc_period);		/* max: 0xff, 255s */
		}

		goto rtc_irq_out;

	} else {

		if (need_work) {
			// only work a rtc period
			need_work = false;
		}
	}

	if (cnt_rtc_min % (SRC_TX_PERIOD/RTC_PERIOD) == (SRC_TX_PERIOD/RTC_PERIOD)-1) {

		rtc_period = RTC_PERIOD - 3;

	} else if (cnt_rtc_min % (SRC_TX_PERIOD/RTC_PERIOD) == 0) {

		need_work = true;

		rtc_period = RTC_PERIOD + 3;

	} else {

		rtc_period = RTC_PERIOD;
	}

	pcf8563_init(SCL_PIN, SDA_PIN);
	pcf8563_clear_timer();
	pcf8563_set_timer_s(rtc_period);

	INFOLN(cnt_rtc_min);

rx_out:

	if (need_work == false) {
		power_off_dev();
	} else {
		power_on_dev();
		radio_setup();
		lora.enter_rx();
		start_ts = seconds();
	}

rtc_irq_out:
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

inline void radio_setup()
{
	lora.reset();
	lora.init();
	lora.setup_v0(TXRX_CH, TX_PWR);
}

void setup()
{
#ifdef ENABLE_CRYPTO
	crypto_init();
#endif

	flash_init();

	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = wdogPeriod_256k;	/* 256k 1kHz periods should give 256 seconds */

	/* Start watchdog */
	WDOG_Init(&wInit);

#ifdef DEBUG
	Serial.setRouteLoc(1);
	Serial.begin(115200);
#endif

	pinMode(PWR_CTRL_PIN, OUTPUT);
	power_on_dev();

	// Key connected to D0
	pinMode(KEY_PIN, INPUT);
	attachInterrupt(KEY_PIN, key_irq_handler, FALLING);

	// RF Interrupt pin
//	pinMode(RF_INT_PIN, INPUT);
//	attachInterrupt(RF_INT_PIN, rx_irq_handler, RISING);

	pcf8563_init(SCL_PIN, SDA_PIN);
	delay(1000);

#if 0
	int ctrl = pcf8563_get_ctrl2();
	INFO("ctrl2: ");
	INFOHEX(ctrl);
	INFOLN("");
#endif

#if 0
	if (ctrl == 0xFF) {
		/* Incorrect state of pcf8563 */
		NVIC_SystemReset();
	}
#endif

	if (g_cfg.init_flag != 0x55aa) {
		/* first bootup */
		pcf8563_set_from_seconds(4 + get_prog_ts());
	}

	/* stop the rtc timer */
	pcf8563_clear_timer();

#if 0
	INFO("ctrl2: ");
	INFOHEX(pcf8563_get_ctrl2());
	INFOLN("");
#endif

	uint32_t cur_ts = pcf8563_now();

	INFOLN(cur_ts);

	pinMode(RTC_INT_PIN, INPUT);
	attachInterrupt(RTC_INT_PIN, rtc_irq_handler, FALLING);

	radio_setup();

	if (g_cfg.paired_did <= 99999999999ULL && g_cfg.paired_did > 0ULL) {
		/*
		 * paired is ok
		 * wdt reset, need to restore the timer
		*/
		need_work = false;
		need_paired = false;

		uint32_t delta = SRC_TX_PERIOD - (cur_ts - g_cfg.paired_rx_ts) % SRC_TX_PERIOD - 6;
		pcf8563_set_timer_s(delta);

		INFOLN("paired");
	} else {
		// pair is not ok, need to start pair

		need_work = true;
		need_paired = true;
		start_ts = seconds();

		lora.enter_rx();

		INFOLN("Wait pair...");
	}

	need_rpt = true;
	set_cc_rpt();
	tx_cause = RESET_TX;
	push_pkt(&g_cbuf, rpt_pkt, 0, PAYLOAD_LEN+6);
}

void deep_sleep()
{
	lora.set_standby(SX126X_STANDBY_RC);
	delay(5);
	lora.set_sleep();

	// dev power off
	power_off_dev();

	//wire_end();
	//digitalWrite(SCL_PIN, HIGH);
	//digitalWrite(SDA_PIN, HIGH);
}

struct pkt d;

void loop()
{
	if (need_rpt == true) {

		INFO("rptx");

		WDOG_Feed();

		power_on_dev();
		radio_setup();

		set_cc_rpt();
		tx_pkt(rpt_pkt, PAYLOAD_LEN+6);

		need_rpt = false;
	}

//	if (need_work == true && 1 == digitalRead(KEY_PIN)
	if (need_work == true
		&& vbat_low == false) {

		WDOG_Feed();

		//INFO("wk");

		rx_worker();
		cc_worker();

		if (need_paired) {
			if (seconds() > (start_ts + pair_win)) {
				need_paired = false;
				need_work = false;
				start_ts = 0;
				INFOLN("pair end");
			}

		} else {

			if (seconds() > (start_ts + work_win)) {
				need_work = false;
				start_ts = 0;
			}
		}
	}

	if (g_cfg.tx_cnt % 100 == 0) {
		/* tx_cnt = 100x, about 1000min */
		g_cfg.init_flag = 0x55aa;

		flash_update();
	}

	if (rtc_period > 240) {
		WDOG_Enable(0);
	} else {
		WDOG_Enable(1);
	}

//	if (need_work == false || 0 == digitalRead(KEY_PIN)
	if (need_work == false
		|| vbat_low == true) {

		WDOG_Feed();

		deep_sleep();

		INFO(".");

		EMU_EnterEM2(true);
	}
}

void rx_worker()
{
	int len = lora.rx(pbuf, 48);
	int rssi = lora.get_pkt_rssi();
	uint32_t ptx_ts = 0;

	if (len > PKT_LEN || len < 0) return;

	if (is_my_pkt(pbuf, len) && need_work) {

		if (need_paired && pbuf[15] == KEY_TX) {
			/* TODO */
			//ptx_ts = pbuf[18] << 24 | pbuf[19] << 16 |
			//		 pbuf[24] << 8 | pbuf[25];
			//rtc_period = ptx_ts + SRC_TX_PERIOD - pcf8563_now();

			ptx_ts = (uint32_t)(pbuf[22] << 8 | pbuf[23]);
			if (ptx_ts < SRC_TX_PERIOD) {
				rtc_period = SRC_TX_PERIOD - ptx_ts - 15;
			} else {
				/* invalid delta time */
				return;
			}

			if (rtc_period > 0 && rtc_period <= 255) {

				pcf8563_set_timer_s(rtc_period);		/* max: 0xff, 255s */
			} else if (rtc_period > 255) {
				/* rtc_period: (255, 600) */
				pcf8563_set_timer(rtc_period/60);			/* max: 0xff, 255min */
			}

			/*
			 * close the pair win
			 * waitting for next rtc irq
			*/
			start_ts = seconds() - pair_win;

			INFOLN(ptx_ts);
			return;
		}

		if (pbuf[15] == TIMER_TX) {
			/*
			 * only accept the rpt by timer
			 * It's the valid paired pkt
			 * need_paired or normal mode
			*/
			if (need_paired || g_cfg.paired_did == get_devid(pbuf)) {

				sync_rtc();

				/* devid is the white id, reset the timer */
				g_cfg.paired_rx_ts = pcf8563_now();
				g_cfg.rx_cnt++;

				if (need_paired) {
					g_cfg.init_flag = 0x55aa;
					g_cfg.paired_did = get_devid(pbuf);

					//flash_update();

					/*
					 * paired, close the pair win
					 * need 2s to re-tx the pkt
					*/
					start_ts = seconds() - pair_win + 2;
				}

				push_pkt(&g_cbuf, pbuf, rssi, len);
			}
			/* do not accept the key rpt or reset rpt */
		}
	}
}

void cc_worker()
{
	noInterrupts();
	int ret = get_pkt(&g_cbuf, &d);
	interrupts();

	if (ret != 0) {
		// there is no pkt
		// Serial.print(".");
	} else {
		tx_pkt(d.data, d.plen);

		#ifdef DEBUG
		hex_pkt(d.data, d.rssi, d.plen);
		#endif

		if (false == need_paired && 0 == get_pkt_cnt(&g_cbuf)) {
			/* cbuf is null, close the tx window */
			need_work = false;
		}
	}
}

uint8_t get_myid_low2()
{
	uint64_t devid = read_devid();

	uint64_t tt = (uint64_t)(devid / 100) * 100;

	uint8_t ret = (uint8_t)(devid - tt);

	return ret;
}

int tx_pkt(uint8_t *p, int len)
{
	if (p[27] != 0) {
		/*
		 * It's not the cc report pkt
		 * Need to process it
		*/
		p[27] |= 0x10;		// mark as cc-relayed
		p[17] = 0;			// don't increment the cc-relayed-cnt

		p[18] = get_myid_low2();
		p[19] = 0;
		p[24] = 0;
		p[25] = 0;
		p[26] = 0;

		uint16_t ui16 = get_crc(p, len-6);
		uint8_t *pcrc = (uint8_t *) &ui16;
		p[len-6] = pcrc[1]; p[len-5] = pcrc[0];

		#ifdef ENABLE_CRYPTO
		set_pkt_mic(p, len);
		#endif
	}

	lora.set_standby(SX126X_STANDBY_RC);

	lora.enable_cad();
	int e = lora.send(p, len, SX126x_TXMODE_SYNC);

	lora.enter_rx();

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

	uint8_t xbit = 0;

	return ret >= 0 ? (ret + xbit) : (ret - xbit);
}

uint16_t get_encode_vbat()
{
	/*
	 *  1: encode the tx_cnt/min
	 * 10: encode the tx_cnt/min
	*/
	float vbat = fetch_vbat();
	uint16_t ui16 = vbat * 100;

	ui16 *= 10;

	if (tx_cnt_1h > 9) {
		tx_cnt_1h = 9;
	}

	ui16 += (uint16_t)tx_cnt_1h;

	return ui16;
}

void set_cc_rpt()
{
	uint8_t *pkt = rpt_pkt;

	memset(pkt, 0, 32);
	pkt[0] = 0x47; pkt[1] = 0x4F; pkt[2] = 0x33;

	uint64_t devid = read_devid();
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

	uint8_t *ep;
	if (g_cfg.paired_did <= 99999999999ULL && g_cfg.paired_did > 12026109999ULL) {
		devid = g_cfg.paired_did;
	} else {
		devid = 0;
	}
	ep = (uint8_t *)&devid;
	pkt[20] = ep[3];
	pkt[21] = ep[2];
	pkt[22] = ep[1];
	pkt[23] = ep[0];

	pkt[27] = 0;		/* data up pkt, no crypto */

	p = (uint8_t *) &(g_cfg.tx_cnt);
	pkt[PAYLOAD_LEN-2] = p[1]; pkt[PAYLOAD_LEN-1] = p[0];
	g_cfg.tx_cnt++;

	ui16 = get_crc(pkt, PAYLOAD_LEN);
	p = (uint8_t *) &ui16;
	pkt[PAYLOAD_LEN] = p[1]; pkt[PAYLOAD_LEN+1] = p[0];

	#ifdef ENABLE_CRYPTO
	set_pkt_mic(pkt, PAYLOAD_LEN+6);
	#endif
}
