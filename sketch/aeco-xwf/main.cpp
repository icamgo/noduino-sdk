/*
 *  Copyright (c) 2019 - 2029 MaiKe Labs
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
*/

#include "softi2c.h"
#include "pcf8563.h"

#include "pt1000.h"

#include "rtcdriver.h"
#include "math.h"
#include "em_wdog.h"

#include "flash.h"

//#define	DEBUG					1
//#define ENABLE_CRC					1

#ifdef CONFIG_V0
#include "softspi.h"
#include "sx1272.h"
#elif USE_SX126X
#include "sx126x.h"
#define SX126X_RST				9
SX126x sx126x(2,					// Pin: SPI CS,PIN06-PB08-D2
				SX126X_RST,			// Pin: RESET, PIN18-PC15-D9
				5,					// PIN: Busy,  PIN11-PB14-D5, The RX pin
				3					// Pin: DIO1,  PIN08-PB11-D3
);
#endif

#define USE_INTERNAL_RTC			1
#define ENABLE_CRYPTO				1
#define ENABLE_CAD					1

#ifdef ENABLE_CRYPTO
#include "crypto.h"
#endif

#define FW_VER					"Ver 1.5"

#ifdef USE_INTERNAL_RTC
/* Timer used for bringing the system back to EM0. */
RTCDRV_TimerID_t xTimerForWakeUp;
#endif

#define LEVEL_UNKNOWN				-2
#define LEVEL_LOW					-1
#define LEVEL_MEDIAN				0
#define LEVEL_HIGH					1

static float cur_temp = 0.0;
static int8_t cur_water = LEVEL_UNKNOWN;

#define	T_FIX					0

static uint32_t need_push = 0;

#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define	KEY_PIN					0		/* PIN01_PA00_D0 */

#define	RTC_INT_PIN				16		/* PIN21_PF02_D16 */

#define SDA_PIN					12		/* PIN23_PE12 */
#define SCL_PIN					13		/* PIN24_PE13 */

#define	RESET_TX			0
#define	DELTA_TX			1
#define	TIMER_TX			2
#define	KEY_TX				3
#define	EL_TX				4
#define	WL_TX				5

uint8_t tx_cause = RESET_TX;


#ifdef CONFIG_V0
#define TXRX_CH					CH_01_472
#define MAX_DBM					20
#define	TX_TIME					1800
#elif USE_SX126X
#define TXRX_CH					472500000
#define MAX_DBM					22
#endif

#define	PAYLOAD_LEN					30		/* 30+2+4 = 36B */
uint8_t message[PAYLOAD_LEN+6] __attribute__((aligned(4)));

bool need_flash __attribute__((aligned(4))) = false;
//uint32_t tx_ts __attribute__((aligned(4))) = 0;


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


#define RTC_PERIOD					30
#define TX_PERIOD					600
#define INIT_TS						1614665566UL
#define MAX_TS						2000000000UL
uint32_t cnt_rtc_min __attribute__((aligned(4))) = 0;
uint32_t rtc_period __attribute__((aligned(4))) = RTC_PERIOD;
uint32_t rtc_ok __attribute__((aligned(4))) = false;
uint32_t cur_ts __attribute__((aligned(4))) = 0;
uint32_t cnt_min __attribute__((aligned(4))) = 0;
uint32_t old_tx_cnt __attribute__((aligned(4))) = 0;

#define LOW_BAT_THRESHOLD			3.4
static float cur_vbat = 0.0;
bool vbat_low __attribute__((aligned(4))) = false;
int cnt_vbat_low __attribute__((aligned(4))) = 0;
int cnt_vbat_ok __attribute__((aligned(4))) = 0;

void push_data();
void sync_tx_ts();

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

float fetch_mcu_temp()
{
	float temp = 0.0;
	for(int i=0; i<3; i++) {
		temp += adc.temperatureCelsius();
	}
	temp /= 3.0;

	return temp;
}

float fetch_vbat()
{
	float vbat = 0;

	for (int i = 0; i < 5; i++) {
		vbat += adc.readVbat();
	}

	return vbat/5.0;
}

int get_water()
{
	int ret = 0;
	uint32_t rt = 0;

	pt1000_init();
	rt = pt1000_get_rt();

	/* 1 - LOW, 2 - Median, 0 - High */
	ret = rt / 10000;

	switch (ret) {
		case 1:
			ret = LEVEL_LOW;
			break;
		case 2:
			ret = LEVEL_MEDIAN;
			break;
		case 0:
			ret = LEVEL_HIGH;
			break;
		default:
			ret = LEVEL_UNKNOWN;
	}

	/* 2/3 x pt1000, (6693, 9234], (10039.5, 13851] */
	if (ret == LEVEL_HIGH && (rt > 9234 || rt <= 6693)) {

		ret = LEVEL_UNKNOWN;
	}

	/* 2 x pt1000, (20078, 27702], (10039, 13851] */
	if (ret == LEVEL_MEDIAN && (rt > 27702 || rt <= 20078)) {

		ret = LEVEL_UNKNOWN;
	}

	/* 1 x pt1000, (10039, 13851] */
	if (ret == LEVEL_LOW && (rt > 13851 || rt <= 10039)) {

		ret = LEVEL_UNKNOWN;
	}

	return ret;
}

float get_temp()
{
	int n = 0;
	uint32_t rt = 0;

	pt1000_init();
	rt = pt1000_get_rt();

	n = rt / 10000;

	if (2 == n) {

		rt /= 2.0;	

	} else if (0 == n) {

		rt *= 1.5;

	} // n >= 3, rt = rt

	// n == 1, rt = rt

	if (n != 3 && (rt > 13851 || rt <= 10039)) {

		// n = 0, 1, 2, 4...

		return -2.0;
	}

	// n = 3 is the 300'C, no sensor connected

	return cal_temp(rt);
}

void key_irq_handler()
{
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);

	need_push = 0x5a;
	tx_cause = KEY_TX;
	g_cfg.tx_cnt++;

	if (g_cfg.tx_ts > INIT_TS && (g_cfg.tx_cnt % 3 == 0)) {
		need_flash = true;
	}

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

	if (rtc_ok == false) {
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
			pcf8563_set_timer_s(rtc_period);

			rtc_ok = true;
			cnt_rtc_min = 0;

			INFOLN(rtc_period);

		} else if ((rtc_period > 255) && (pps > 0)) {

			/* pps: (0, 59] */
			pps = rtc_period / 60;
			rtc_period = 60 * pps;

			/* waiting for first tx */
			pcf8563_set_timer(pps);		/* max: 0xff, 255s */

			INFO("m: ");
			INFOLN(pps);

			goto out;
		}
	}

	pcf8563_init(SCL_PIN, SDA_PIN);

	#ifdef DEBUG_RTC
	INFO("ctrl2: ");
	INFOHEX(pcf8563_get_ctrl2());
	INFOLN("");

	INFO("irq_ts: ");
	INFOLN(pcf8563_now());
	#endif

	pcf8563_reset_timer();

	if (cnt_rtc_min % (TX_PERIOD/RTC_PERIOD) == 0) {
		/* 10 min */
		need_push = 0x5a;
		tx_cause = TIMER_TX;

		g_cfg.tx_cnt++;
		g_cfg.tx_ts = pcf8563_now();

		if (0x55aa != g_cfg.init_flag) {
			/* first tx */
			need_flash =true;
		}
	}

out:
	#ifndef USE_INTERNAL_RTC
	if (rtc_period > 100) {
		WDOG_Enable(0);
	} else {
		WDOG_Enable(1);
	}
	#endif

	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

#ifdef USE_INTERNAL_RTC
void period_check(RTCDRV_TimerID_t id, void *user)
{
	/* reset the watchdog */
	WDOG_Feed();

	cnt_min++;

	if (cnt_min % 60 == 0) {
		/* check the tx_cnt */

		if (g_cfg.tx_cnt - old_tx_cnt <= 1) {
			/*
			 * seems like it's not ok
			 * need to re-sync the rtc
			*/
			rtc_ok = false;

			cur_ts = pcf8563_now();
			rtc_period = TX_PERIOD - cur_ts % TX_PERIOD;
			INFOLN("re-sync rtc");
			sync_tx_ts();
		}

		old_tx_cnt = g_cfg.tx_cnt;
	}
}
#endif

void setup()
{
	Ecode_t e;

	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = wdogPeriod_128k;	/* 32k 1kHz periods should give 4 seconds */

	/* Start watchdog */
	WDOG_Init(&wInit);

	flash_init();

#ifdef ENABLE_CRYPTO
	crypto_init();
#endif

	// init dev power ctrl pin
	pinMode(PWR_CTRL_PIN, OUTPUT);
	power_off_dev();

	pinMode(KEY_PIN, INPUT);
	attachInterrupt(KEY_PIN, key_irq_handler, FALLING);

#ifdef DEBUG
	Serial.setRouteLoc(1);
	Serial.begin(115200);
#endif

	//power_on_dev();		/* To consume the current ? */
	pcf8563_init(SCL_PIN, SDA_PIN);

	//i2c_delay_ms(1000);
	delay(1000);

#ifdef DEBUG_RTC
	int ctrl = pcf8563_get_ctrl2();
	INFO("RTC ctrl2: ");
	INFOHEX(ctrl);
	INFOLN("");

	if (ctrl == 0xFF) {
		/* Incorrect state of pcf8563 */
		NVIC_SystemReset();
	}
#endif

	pcf8563_clear_timer();

	pinMode(RTC_INT_PIN, INPUT_PULLUP);
	attachInterrupt(RTC_INT_PIN, rtc_irq_handler, FALLING);

	if (g_cfg.init_flag != 0x55aa) {
		/*
		 * First bootup
		*/
		cur_ts = get_prog_ts() + 4;
		pcf8563_set_from_seconds(cur_ts);

	} else {
		cur_ts = pcf8563_now();
	}

	INFOLN(cur_ts);

	if (cur_ts > INIT_TS && cur_ts < MAX_TS) {
		/*
		 * cur_ts is valide, align the tx_ts to TX_PERIOD
		*/
		rtc_period = TX_PERIOD - cur_ts % TX_PERIOD;

		INFOLN("init tx_ts");
		sync_tx_ts();

	} else {
		/*
		 * cur_ts is invalide
		 * try to restore from flash
		*/
		#if 0
		int delta = (cur_ts - g_cfg.tx_ts);

		if (delta > 0 && delta < 1800) {

			int dd = delta % TX_PERIOD;

			if (dd > 0) {

				rtc_period = TX_PERIOD - dd;

			} else {
				/* dd == 0 */
				rtc_period = RTC_PERIOD;
				rtc_ok = true;
				pcf8563_set_timer_s(rtc_period);
				goto setup_out;
			}

			INFOLN("load tx_ts");

		} else {
		#endif
			/*
			 * Exception: rtc or tx_ts
			 * reset the tx_ts
			*/
			cur_ts = get_prog_ts() + 4;

			pcf8563_set_from_seconds(cur_ts);

			g_cfg.tx_ts = 0;
			g_cfg.init_flag = 0x55aa;

			flash_update();

			rtc_period = TX_PERIOD - cur_ts % TX_PERIOD;

			INFOLN("cur_ts is invalide");
		//}

		sync_tx_ts();
	}

	//power_off_dev();

setup_out:
	/* bootup tx */
	tx_cause = RESET_TX;
	need_push = 0x5a;
	g_cfg.tx_cnt++;

	#ifdef USE_INTERNAL_RTC
	/* Initialize RTC timer. */
	RTCDRV_Init();
	RTCDRV_AllocateTimer(&xTimerForWakeUp);
	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypePeriodic, 60 * 1000, period_check, NULL);
	#endif
}

void qsetup()
{
#ifdef CONFIG_V0
	sx1272.setup_v0(TXRX_CH, MAX_DBM);
	#ifdef ENABLE_CAD
	sx1272._enableCarrierSense = true;
	#endif

#elif USE_SX126X
	sx126x.wakeup();
	sx126x.init();
	sx126x.setup_v0(TXRX_CH, MAX_DBM);
#endif
}

#ifdef ENABLE_CRC
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

void push_data()
{
	long startSend;
	long endSend;

	int e;

	uint8_t *pkt = message;

	memset(pkt, 0, PAYLOAD_LEN+6);
	pkt[0] = 0x47; pkt[1] = 0x4F;

	////////////////////////////////
	pkt[15] = tx_cause;
	////////////////////////////////

	cur_vbat = fetch_vbat();

	power_on_dev();
	cur_temp = get_temp() + T_FIX;
	cur_water = get_water();

	uint64_t devid = get_devid();

	uint8_t *p = (uint8_t *) &devid;

	// set devid
	int i = 0;
	for(i = 0; i < 8; i++) {
		pkt[3+i] = p[7-i];
	}

	int16_t i16 = (int16_t)(cur_temp * 10);

	p = (uint8_t *) &i16;
	pkt[11] = p[1]; pkt[12] = p[0];

	i16 = cur_vbat * 1000;
	pkt[13] = p[1]; pkt[14] = p[0];

	pkt[15] = tx_cause;

	pkt[2] = 0x33;

	float chip_temp = fetch_mcu_temp();

	// Humidity Sensor data	or Water Leak Sensor data
	pkt[20] = cur_water;

	// Internal Temperature of the chip
	pkt[21] = (int8_t)roundf(chip_temp);

	// Internal humidity to detect water leak of the shell
	//pkt[22] = cur_water;

	// Internal current consumption
	//pkt[23] = cur_water;

	// pkt[27] = 0b100, set bit 2, HTimer rpt pkt
	pkt[PAYLOAD_LEN-3] = 0x4;

	cur_ts = pcf8563_now();

	if (tx_cause == RESET_TX) {

		p = (uint8_t *) &(cur_ts);

	} else {

		if (g_cfg.tx_ts < INIT_TS) {
			g_cfg.tx_ts = 0;
		}

		p = (uint8_t *) &(g_cfg.tx_ts);
	}

	pkt[18] = p[3];
	pkt[19] = p[2];
	pkt[24] = p[1];
	pkt[25] = p[0];

	cur_ts -= g_cfg.tx_ts;
	if (cur_ts > 600) {
		cur_ts = 600;
	}
	p = (uint8_t *) &cur_ts;
	pkt[22] = p[1];
	pkt[23] = p[0];

	p = (uint8_t *) &(g_cfg.tx_cnt);
	pkt[PAYLOAD_LEN-2] = p[1]; pkt[PAYLOAD_LEN-1] = p[0];

	/////////////////////////////////////////////////////////
	/*
	 * 2. crc
	 * 3. set mic
	*/
	#ifdef ENABLE_CRC
	uint16_t ui16 = get_crc(pkt, PAYLOAD_LEN);
	p = (uint8_t *) &ui16;
	pkt[PAYLOAD_LEN] = p[1]; pkt[PAYLOAD_LEN+1] = p[0];
	#else
	pkt[PAYLOAD_LEN] = 0; pkt[PAYLOAD_LEN+1] = 0;
	#endif

#ifdef ENABLE_CRYPTO
	set_pkt_mic(pkt, PAYLOAD_LEN+6);
#endif
	/////////////////////////////////////////////////////////

	qsetup();

#ifdef DEBUG
	startSend = millis();
#endif

#ifdef CONFIG_V0

	#ifdef ENABLE_CAD
	sx1272.CarrierSense();
	#endif

	e = sx1272.sendPacketTimeout(1, message, PAYLOAD_LEN+6, TX_TIME);

	sx1272.setSleepMode();
	digitalWrite(SX1272_RST, LOW);
	spi_end();
#elif USE_SX126X

	#ifdef ENABLE_CAD
	sx126x.enable_cad();
	#endif
	e = sx126x.send(message, PAYLOAD_LEN+6, SX126x_TXMODE_SYNC);
	sx126x.set_sleep();

	power_off_dev();
	//digitalWrite(SX126X_RST, LOW);
#endif

	if (!e) {
		// send message succesful, update the old data
	}

#ifdef DEBUG
	endSend = millis();

	INFO("TX time: ");
	INFOLN(endSend - startSend);
#endif

	INFO("TX state: ");
	INFOLN(e);
}

void loop()
{
	if (0x5a == need_push) {

		WDOG_Feed();

		push_data();

		need_push = 0;
	}

	if (g_cfg.tx_cnt % 100 == 0 || need_flash) {
		/* tx_cnt = 100x, about 1000min */
		g_cfg.init_flag = 0x55aa;
		g_cfg.epoch = pcf8563_now();
		flash_update();

		need_flash = false;
	}

#if defined(CONFIG_V0)
	digitalWrite(SX1272_RST, LOW);
	spi_end();
#endif

	//wire_end();
	//digitalWrite(SCL_PIN, HIGH);
	//digitalWrite(SDA_PIN, HIGH);

	power_off_dev();

	EMU_EnterEM2(true);
}

void sync_tx_ts()
{
	int s = 0;

	if (rtc_period > 0 && rtc_period < 255) {

		pcf8563_set_timer_s(rtc_period);		/* max: 0xff, 255s */
		INFO("s: ");
		INFOLN(rtc_period);

	} else if (rtc_period == TX_PERIOD) {
		rtc_ok = true;
		rtc_period = RTC_PERIOD;
		pcf8563_set_timer_s(rtc_period);

	} else {
		/* rtc_period: (255, 600) */
		s = rtc_period % 60;
		pcf8563_set_timer_s(s);			/* max: 0xff, 255min */
		INFO("-m: ");
		INFOLN(s);
		INFOLN(rtc_period);
	}

#ifndef USE_INTERNAL_RTC
	if (rtc_period > 100) {
		WDOG_Enable(0);
	} else {
		WDOG_Enable(1);
	}
#endif
}
