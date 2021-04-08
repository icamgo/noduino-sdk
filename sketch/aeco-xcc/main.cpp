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
#include "crypto.h"

#include "tx_ctrl.h"
#include "circ_buf.h"
#include "flash.h"

#define DEBUG			1

#define RTC_PERIOD		30

#define TXRX_CH			472500000	// Hz center frequency
#define TX_PWR			22			// dBm tx output power

////////////////////////////////////////////////////////////
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

SX126x lora(2,		// Pin: SPI CS,PIN06-PB08-D2
	    9,				// Pin: RESET, PIN18-PC15-D9
	    5,				// PIN: Busy,  PIN11-PB14-D5, The RX pin
	    3				// Pin: DIO1,  PIN08-PB11-D3
    );

struct circ_buf g_cbuf __attribute__((aligned(4)));
struct ctrl_fifo g_cfifo __attribute__((aligned(4)));

int32_t cnt_rtc_min __attribute__((aligned(4))) = 0;
uint32_t rtc_period __attribute__((aligned(4))) = RTC_PERIOD;
bool need_work __attribute__((aligned(4))) = true;

bool vbat_low __attribute__((aligned(4))) = false;

bool need_paired __attribute__((aligned(4))) = false;

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

inline uint64_t get_devid()
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

	if (20 == dtype || 21 == dtype) {
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
}

uint8_t pbuf[48];

void rx_irq_handler()
{
	int len = 0, rssi = 0;

	INFOLN("rx");

	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);

	len = lora.get_rx_pkt(pbuf, 48);
	rssi = lora.get_pkt_rssi();

	if (len > PKT_LEN || len < 0) goto irq_out;

	if (is_my_pkt(pbuf, len) && need_work) {
		// if the devid is the white id
		// reset the timer
		sync_rtc();

		push_pkt(&g_cbuf, pbuf, rssi, len);
	}

irq_out:

	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

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

	Serial.print("/");
	Serial.println(check_pkt_mic(p, plen));
}

void key_irq_handler()
{
	/*
	 * 1. report white list id
	 * 2. waiting 30s? for the new white list id
	*/
	power_on_dev();

	WDOG_Feed();

	pcf8563_clear_timer();

	lora.enter_rx();
	need_work = true;

	INFOLN("key");

	INFOHEX(pcf8563_get_ctrl2());
	INFOLN("");

	INFOHEX(pcf8563_get_timer());
	INFOLN("");
}

void radio_setup();

void rtc_irq_handler()
{
#if 1
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
#endif
	power_on_dev();

	WDOG_Feed();

	cnt_rtc_min++;

	if (need_work) {
		// only work a rtc period
		need_work = false;
	}

	if (cnt_rtc_min % 20 == 19) {

		rtc_period = 25;


	} else if (cnt_rtc_min % 20 == 0) {

		power_on_dev();

		//radio_setup();

		lora.enter_rx();

		need_work = true;

		rtc_period = 35;

	} else {

		rtc_period = RTC_PERIOD;
	}

	pcf8563_init(SCL_PIN, SDA_PIN);

	i2c_delay(50*I2C_1MS);
	pcf8563_clear_timer();
	pcf8563_set_timer_s(rtc_period);

	INFO("rtc, ");
	INFOLN(cnt_rtc_min);


	INFOHEX(pcf8563_get_ctrl2());
	INFOLN("");

	INFOHEX(pcf8563_get_timer());
	INFOLN("");

	if (need_work == false) {
		power_off_dev();
	}
	//need_push = 0x5a;
	//tx_cause = TIMER_TX;

#if 1
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
#endif
}

inline void radio_setup()
{
#if 1
	lora.reset();
#else
	lora.wakeup();
#endif
	lora.init();
	lora.setup_v0(TXRX_CH, TX_PWR);
}

void setup()
{
	crypto_init();

	flash_init();

	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = wdogPeriod_64k;	/* 256k 1kHz periods should give 256 seconds */

	/* Start watchdog */
	WDOG_Init(&wInit);

#ifdef DEBUG
	Serial.setRouteLoc(1);
	Serial.begin(115200);
#endif

	pinMode(PWR_CTRL_PIN, OUTPUT);

	// Key connected to D0
	pinMode(KEY_PIN, INPUT);
	attachInterrupt(KEY_PIN, key_irq_handler, FALLING);

	// RF Interrupt pin
	pinMode(RF_INT_PIN, INPUT);
	attachInterrupt(RF_INT_PIN, rx_irq_handler, RISING);

	pcf8563_init(SCL_PIN, SDA_PIN);

	int ctrl = pcf8563_get_ctrl2();
	INFO("RTC ctrl2: ");
	INFOHEX(ctrl);
	INFOLN("");

	if (ctrl == 0xFF) {
		while(true) {
		}
	}

	pcf8563_set_from_seconds(get_prog_ts());
	//sync_rtc();

	INFO("RTC ctrl2: ");
	INFOHEX(pcf8563_get_ctrl2());
	INFOLN("");
	INFOLN(pcf8563_now());
	INFO("RTC timer: ");
	INFOHEX(pcf8563_get_timer());
	INFOLN("");

	pinMode(RTC_INT_PIN, INPUT);
	attachInterrupt(RTC_INT_PIN, rtc_irq_handler, FALLING);

	power_on_dev();

	radio_setup();
	lora.enter_rx();

	Serial.println("RX testing...");
}

void deep_sleep()
{
	lora.set_sleep();

	// dev power off
	power_off_dev();

	//wire_end();
	digitalWrite(SCL_PIN, HIGH);
	digitalWrite(SDA_PIN, HIGH);
}

struct pkt d;

void cc_worker();

void loop()
{
//	if (need_work == true && 1 == digitalRead(KEY_PIN)
	if (need_work == true
		&& vbat_low == false) {

		cc_worker();
	}

//	if (need_work == false || 0 == digitalRead(KEY_PIN)
	if (need_work == false
		|| vbat_low == true) {

		WDOG_Feed();

		deep_sleep();

		INFOLN("s");

		EMU_EnterEM2(true);
	}
}

int tx_pkt(uint8_t *p, int len);

void cc_worker()
{
	WDOG_Feed();

	int len = lora.rx(pbuf, 48);
	int rssi = lora.get_pkt_rssi();

	if (len > PKT_LEN || len < 0) return;

	if (is_my_pkt(pbuf, len) && need_work) {
		// if the devid is the white id
		// reset the timer
		sync_rtc();

		push_pkt(&g_cbuf, pbuf, rssi, len);
	}

	noInterrupts();
	int ret = get_pkt(&g_cbuf, &d);
	interrupts();

	if (ret != 0) {
		// there is no pkt
		// Serial.print(".");
	} else {

		tx_pkt(d.data, d.plen);

		hex_pkt(d.data, d.rssi, d.plen);
		need_work = false;
	}
}

int tx_pkt(uint8_t *p, int len)
{
	p[27] |= 0x10;		// mark as cc-relayed
	p[17] = 1;			// increment the cc-relayed-cnt

	uint16_t ui16 = get_crc(p, len-6);
	uint8_t *pcrc = (uint8_t *) &ui16;
	p[len-6] = pcrc[1]; p[len-5] = pcrc[0];

	set_pkt_mic(p, len);

	lora.set_standby(SX126X_STANDBY_RC);

	lora.enable_cad();
	int e = lora.send(p, len, SX126x_TXMODE_SYNC);

	lora.enter_rx();

	return e;
}
