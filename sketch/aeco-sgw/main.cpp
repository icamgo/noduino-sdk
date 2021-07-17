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

#include "m5311.h"

extern "C"{
#include "timex.h"
}

#include "rtcdriver.h"
#include "math.h"
#include "em_wdog.h"

#include "sx126x.h"
#include "circ_buf.h"


#define ENABLE_NB						1
#define ENABLE_LORA						1
//#define ENABLE_RX_IRQ					1

#define DEBUG							1
#define DEBUG_HEX_PKT					1
//#define ENABLE_RTC						1
//#define DEBUG_RTC						1

#define	FW_VER						"V1.2"

#define LOW_BAT_THRESHOLD			3.0

#include "softi2c.h"
#ifdef ENABLE_RTC
#include "pcf8563.h"
#endif

/* Timer used for bringing the system back to EM0. */
RTCDRV_TimerID_t xTimerForWakeUp;

static uint32_t check_period = 54;		/* 60s */

#define WDOG_PERIOD				wdogPeriod_64k			/* 32k 1kHz periods should give 32 seconds */

#define PUSH_PERIOD				1080	/* check_period x PUSH_PERIOD = 360min, 6h */
#define	MY_RPT_MSG1			"{\"gid\":\"%s\"`\"B\":%s`\"TX\":%d`\"iT\":%d`\"tp\":%d`\"sid\":\"%s\"}"

#define INIT_TS						1614665566UL
#define MAX_TS						2000000000UL

static uint32_t cnt_1min = 0;

#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define MODEM_ON_PIN			0		/* PIN01_PA00_D0 */

#define	KEY_PIN					15		/* PIN20_PF01_D15 */

#define RF_CS_PIN				2		/* PIN6_PB08_D2 */
#define	RF_RST_PIN				5		/* PIN11_PB14_D5, LeUART0_RX */
#define	RF_BUSY_PIN				9		/* PIN18_PC15_D9 */
#define	RF_INT_PIN				3		/* PIN8_PB11_D3, DIO1 */

#define	RTC_INT_PIN				16		/* PIN21_PF02_D16 */

float cur_vbat __attribute__((aligned(4))) = 0.0;
bool vbat_low __attribute__((aligned(4))) = false;
int cnt_vbat_low __attribute__((aligned(4))) = 0;
int cnt_vbat_ok __attribute__((aligned(4))) = 0;

static bool need_sleep __attribute__((aligned(4))) = false;
static bool need_push __attribute__((aligned(4))) = false;
uint32_t cnt_sleep __attribute__((aligned(4))) = 0;

static bool need_init_lora __attribute__((aligned(4))) = false;

uint16_t tx_count = 0;

#define	MQTT_PUSH_MSG			"{\"ts\":%d`\"gwid\":\"%s\"`\"gid\":\"%s\"`\"B\":%s`\"tp\":%d`\"sgi\":%d`\"%s}"

#ifdef DEBUG
#define INFO_S(param)			Serial.print(F(param))
#define INFO_HEX(param)			Serial.print(param,HEX)
#define INFO(param)				Serial.print(param)
#define INFOLN(param)			Serial.println(param)
#define FLUSHOUTPUT				Serial.flush();
#else
#define INFO_S(param)
#define INFO(param)
#define INFOLN(param)
#define FLUSHOUTPUT
#endif

#define	RESET_TX			0
#define	DELTA_TX			1
#define	TIMER_TX			2
#define	KEY_TX				3
#define	EL_TX				4
#define	WL_TX				5

struct circ_buf g_cbuf __attribute__((aligned(4)));

int tx_cause = RESET_TX;

M5311 modem;

bool network_ok __attribute__((aligned(4))) = false;
char iccid[24] __attribute__((aligned(4)));
int g_rssi = 0;
char *myid = NULL;

///////////////////////////////////////////////////////////////
#include "ccx.h"
#include "crypto.h"
#include "tx_ctrl.h"

#define TXRX_CH					472500000
#define TX_PWR					22

uint8_t pbuf[48] __attribute__((aligned(4)));;
struct ctrl_fifo g_cfifo __attribute__((aligned(4)));

SX126x lora(RF_CS_PIN,			// SPI_CS
	    RF_RST_PIN,				// RESET
	    RF_BUSY_PIN,			// BUSY
	    RF_INT_PIN				// DIO1
    );

inline void setup_lora()
{
	lora.init();
	lora.setup_v0(TXRX_CH, TX_PWR);
}

#ifdef ENABLE_RX_IRQ
void rx_irq_handler()
{
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);

	int plen = 0, rssi = 0;

	WDOG_Feed();

	INFO("rx: ");

	plen = lora.get_rx_pkt(pbuf, 48);
	rssi = lora.get_pkt_rssi();

	INFOLN(plen);

	if (plen > PKT_LEN || plen < 24) goto rx_irq_out;

	if ((true == is_our_pkt(pbuf, plen))
		&& (false == is_pkt_in_ctrl(&g_cfifo, pbuf, plen, seconds()))) {

		push_point(&g_cbuf, pbuf, rssi, plen, seconds());

		/*
		 * push the pkt data into tx_ctrl structure
		 * TODO: should do it in the pkt process func
		*/
		check_ctrl_fno(&g_cfifo, pbuf, plen);
	}

rx_irq_out:
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}
#endif
///////////////////////////////////////////////////////////////

void push_data();
int8_t fetch_mcu_temp();

uint64_t get_devid()
{
	uint64_t *p;

	p = (uint64_t *)0x0FE00008;

	return *p;
}

void power_on_dev()
{
	digitalWrite(PWR_CTRL_PIN, HIGH);
}

void power_off_dev()
{
	digitalWrite(PWR_CTRL_PIN, LOW);

	need_init_lora = true;
}

void power_on_modem()
{
	//digitalWrite(PWR_CTRL_PIN, HIGH);

#if 1
	digitalWrite(MODEM_ON_PIN, HIGH);
	delay(2200);
	digitalWrite(MODEM_ON_PIN, LOW);
	delay(200);
#endif
}

void power_off_modem()
{
	digitalWrite(MODEM_ON_PIN, HIGH);
	delay(8300);
	digitalWrite(MODEM_ON_PIN, LOW);
}

void wakeup_modem()
{
	digitalWrite(MODEM_ON_PIN, HIGH);
	delay(100);
	digitalWrite(MODEM_ON_PIN, LOW);
	delay(200);
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
	WDOG_Feed();

	power_off_modem();

	power_off_dev();
	i2c_delay_ms(9000);			/* delay 6000ms */
	NVIC_SystemReset();
}

void period_check_status(RTCDRV_TimerID_t id, void *user)
{
	/* reset the watchdog */
	WDOG_Feed();

	cnt_1min++;

	if (false == vbat_low) {

		if (cnt_1min % 60 == 0) {

			/* 60min Timer */
			need_push = 0x55;
			tx_cause = TIMER_TX;
		}

		#if 0
		if (cnt_1min % 1440 == 0) {
			/* 24h */
			reset_dev_sys();
		}
		#endif
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

		cnt_vbat_ok = 0;

	} else {

		cnt_vbat_ok++;

		if (cnt_vbat_ok >= 10) {

			if (vbat_low) {
				/* Reset the system */
				reset_dev_sys();
			}

			vbat_low = false;
			cnt_vbat_ok = 0;
		}

		cnt_vbat_low = 0;
	}
}

void key_report_status()
{
	/* report via lora */
	need_push = 0x66;
	tx_cause = KEY_TX;
	INFOLN("key");
}

int setup_nb()
{
	int start_cnt = 0;

	power_off_modem();
	power_off_dev();
	delay(3000);
	power_on_dev();

	INFOLN("....");
	SerialUSART1.setRouteLoc(2);
	SerialUSART1.begin(115200);
	INFOLN("xxxx");

	modem.init(&SerialUSART1);

qsetup_start:

	INFOLN(__LINE__);
	WDOG_Feed();
	INFOLN(__LINE__);
	power_on_modem();
	INFOLN(__LINE__);

	delay(1200);
	modem.init_modem();

	memset(iccid, 0, 24);
	modem.get_iccid().toCharArray(iccid, 24);

	int ret = 0;
	ret = modem.check_boot();
	start_cnt++;

	if (ret == 1) {

		network_ok = true;

	} else if ((ret == 2 || ret == 0) && start_cnt < 3) {
		INFOLN(__LINE__);
		delay(1000);
		INFOLN(__LINE__);
		goto qsetup_start;
		INFOLN(__LINE__);
	} else {

		for (int i = 0; i < 15; i++) {

			WDOG_Feed();
			ret = modem.check_network();

			if (ret == 1) {
				network_ok = true;
				break;

			} else {

				INFOLN("Try to wakeup modem");
				wakeup_modem();
			}

			delay(1000);

			INFOLN("network check");

			//modem.init_modem();
		}

	}

	if (network_ok) {

		//modem.disable_deepsleep();

		//INFOLN("IMEI = " + modem.get_imei());
		//INFOLN("IMSI = " + modem.get_imsi());
		//INFOLN(modem.check_ipaddr());

		extern char str[BUF_LEN];
		memset(str, 0, BUF_LEN);

		WDOG_Feed();

		g_rssi = modem.get_csq();
		INFOLN(g_rssi);

		//char strtest[] = "21/02/26,06:22:38+32";
		modem.get_net_time().toCharArray(str, BUF_LEN);
		INFOLN(str);

		uint32_t sec = str2seconds(str);

		if (sec > INIT_TS && sec < MAX_TS) {
			update_seconds(sec);

		#ifdef ENABLE_RTC
			pcf8563_init(SCL_PIN, SDA_PIN);
			pcf8563_set_from_seconds(sec);
		#endif
		}

		INFO("epoch = ");
		INFOLN(seconds());
		#ifdef ENABLE_RTC
		INFOLN(pcf8563_now());
		#endif
	} else {
		/* attach network timeout */
		power_on_modem();
		delay(100);
		modem.clean_net_cache();
		modem.enter_deepsleep();
		delay(2000);
	}

	return network_ok;
}

int mqtt_init()
{
	int ret = 0;

	modem.mqtt_end();

	WDOG_Feed();
	wakeup_modem();
	modem.mqtt_begin("mqtt.autoeco.net", 1883, myid);

	delay(100);
	ret = modem.mqtt_connect();

	return ret;
}

int8_t fetch_mcu_temp()
{
	float temp = 0.0;
	for(int i=0; i<3; i++) {
		temp += adc.temperatureCelsius();
	}
	temp /= 3.0;

	return (int8_t)roundf(temp);
}

void setup()
{
	Ecode_t e;

	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = WDOG_PERIOD;

	crypto_init();

	// dev power ctrl
	pinMode(PWR_CTRL_PIN, OUTPUT);
	pinMode(MODEM_ON_PIN, OUTPUT);

	power_on_dev();
	digitalWrite(MODEM_ON_PIN, LOW);

	pinMode(KEY_PIN, INPUT);
	attachInterrupt(KEY_PIN, key_report_status, FALLING);

	/* Initialize RTC timer. */
	RTCDRV_Init();
	RTCDRV_AllocateTimer(&xTimerForWakeUp);
	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypePeriodic, check_period * 1000, period_check_status, NULL);

#ifdef DEBUG
	Serial.setRouteLoc(1);
	Serial.begin(115200);
#endif

	/* Start watchdog */
	WDOG_Init(&wInit);

	/* reset epoch */
	update_seconds(1625736586);
	reset_ctrl_ts(&g_cfifo, seconds());

	/* boot tx */
	tx_cause = RESET_TX;

	INFOLN("\r\n\r\nWelcom to AE-SGW!");
	INFO("Firmware: ");
	INFOLN(FW_VER);
	INFO("DevID: ");
	myid = uint64_to_str(my_devid, get_devid());
	INFOLN(myid);

	INFO("epoch = ");
	INFOLN(seconds());

#ifdef ENABLE_RTC
	pcf8563_init(SCL_PIN, SDA_PIN);

	delay(1000);

	#ifdef DEBUG_RTC
	int ctrl = pcf8563_get_ctrl2();
	INFO("RTC ctrl2: ");
	INFO_HEX(ctrl);
	INFOLN("");

	if (ctrl == 0xFF) {
		/* Incorrect state of pcf8563 */
		NVIC_SystemReset();
	}
	#endif

	pcf8563_clear_timer();
#endif

#ifdef ENABLE_LORA
#ifdef ENABLE_RX_IRQ
	// RF Interrupt pin
	#if 1
	pinMode(RF_INT_PIN, INPUT);
	#else
	GPIO_PinModeSet(g_Pin2PortMapArray[RF_INT_PIN].GPIOx_Port,
					g_Pin2PortMapArray[RF_INT_PIN].Pin_abstraction,
					gpioModeInputPullFilter, 0);
	#endif

	attachInterrupt(RF_INT_PIN, rx_irq_handler, RISING);
#endif

	setup_lora();
	lora.enter_rx();
#endif

	#ifdef ENABLE_RX_IRQ
	delay(3000);
	lora.set_sleep();
	need_push = true;
	#endif

#ifdef ENABLE_NB
	//setup_nb();
#endif
}

void push_data()
{
	long cnt_fail = 0;
	static uint32_t pub_timeout = 1500;

	struct pkt d;

	WDOG_Feed();

	int ret = get_1st_point(&g_cbuf, &d);

	if (ret == 1) {
		//INFO(".");
		return;
	}

	if (false == network_ok) {

		setup_nb();
		mqtt_init();

	} else {
		/* reset the usart1 */
		SerialUSART1.setRouteLoc(2);
		SerialUSART1.begin(115200);
		wakeup_modem();
	}

	if (network_ok) {

		INFOLN("network is ok");

		WDOG_Feed();
		wakeup_modem();

		while (get_1st_point(&g_cbuf, &d) == 0 && cnt_fail < 8) {

			WDOG_Feed();

			#ifdef ENABLE_RX_IRQ
			if (false == check_pkt_mic(d.data, d.plen)) {
				noInterrupt();
				pop_point(&g_cbuf, &d);
				interrupts();
				continue;
			}
			#endif

			if (d.ts < INIT_TS || d.ts > MAX_TS) {
				INFOLN("Invalid ts, use current ts");
				d.ts = seconds();
				INFOLN(seconds());
			}

			extern char modem_said[MODEM_LEN];
			memset(modem_said, 0, MODEM_LEN);

			#if 0
			if (tx_cause == 0 || gmtime(&(d.ts))->tm_hour == 12) {

				sprintf(modem_said, MY_RPT_MSG,
						d.ts,
						my_devid,
						decode_vbat(vbat),
						decode_sensor_data(d.data / 10.0),
						d.iT,
						tx_cause,
						iccid
				);
			} else {
			}
			#endif

			wakeup_modem();

			sprintf(modem_said, MQTT_PUSH_MSG,
					d.ts,
					myid,
					decode_devid(d.data, devid_buf),
					decode_vbat(d.data),
					decode_cmd(d.data),
					d.rssi,
					decode_sensor_data(d.data)
			);

			ret = modem.mqtt_pub("dev/gws", modem_said, pub_timeout);

			if (ret == 1) {

				INFOLN("Pub OK, pop point");
				#ifdef ENABLE_RX_IRQ
				noInterrupt();
				#endif
				pop_point(&g_cbuf, &d);
				#ifdef ENABLE_RX_IRQ
				interrupts();
				#endif

				pub_timeout = 2200;

			} else if (ret == -1) {
				/* timeout */
				pub_timeout = 6000;

				cnt_fail++;

				INFO("pub timeout: ");
				INFOLN(pub_timeout);

			} else {

				cnt_fail++;
				INFOLN("Pub failed");
			}
		}

		if (cnt_fail >= 8) {
			network_ok = false;
			//modem.mqtt_end();
		}

		INFOLN("no point");

		WDOG_Feed();
	}

	WDOG_Feed();
}

void deep_sleep()
{
	lora.set_standby(SX126X_STANDBY_RC);
	delay(5);
	lora.set_sleep();

	power_off_modem();

	power_off_dev();

	EMU_EnterEM2(true);
}

#ifndef ENABLE_RX_IRQ
void lora_rx_worker()
{
	int plen = lora.rx(pbuf, 48);
	int rssi = lora.get_pkt_rssi();
	uint32_t ptx_ts = 0;
	int ret = 0;

	INFOLN(plen);

	if (plen > PKT_LEN || plen < PKT_LEN_MIN) return;

	if ((true == is_our_pkt(pbuf, plen))
		&& (true == check_pkt_mic(pbuf, plen))
		&& (false == is_pkt_in_ctrl(&g_cfifo, pbuf, plen, seconds()))) {

		ret = push_point(&g_cbuf, pbuf, rssi, plen, seconds());

		if (1 == ret) {
			/* cbuf is full */
			need_push = true;
		}

		/*
		 * push the pkt data into tx_ctrl structure
		 * TODO: should do it in the pkt process func
		*/
		check_ctrl_fno(&g_cfifo, pbuf, plen);
	}

#ifdef DEBUG_HEX_PKT
	int a = 0, b = 0;

	for (; a < plen; a++, b++) {

		if ((uint8_t) pbuf[a] < 16)
			INFO_S("0");

		INFO_HEX((uint8_t) pbuf[a]);
		INFO_S(" ");
	}

	INFO_S("/");
	INFO(rssi);
	INFO_S("/");
	INFOLN(plen);
#endif
}
#endif

void loop()
{
	//if (need_sleep == false && 1 == digitalRead(KEY_PIN) && vbat_low == false) {
	if (false == vbat_low) {

		WDOG_Feed();

	#ifndef ENABLE_RX_IRQ
		lora_rx_worker();
	#endif
		delay(300);

	#ifdef ENABLE_NB
		if (need_push == true) {

			/* clear usart1 to start nb */
			lora.set_sleep();
			lora.spi_end();
			//digitalWritel(RF_RST_PIN, LOW);

			push_data();

			need_push = false;

			#ifndef DONOT_USE_BUSY
			// using the busy pin
			GPIO_PinModeSet(g_Pin2PortMapArray[RF_BUSY_PIN].GPIOx_Port,
							g_Pin2PortMapArray[RF_BUSY_PIN].Pin_abstraction,
							gpioModeInputPullFilter, 0);
			setup_lora();
			#else
			// donot use the busy pin
			if (need_init_lora) {
				setup_lora();
				need_init_lora = false;
			} else {
				lora.spi_init();
			}
			#endif
			lora.enter_rx();
		}
	#endif
	} else {
		// vbat is low
		INFOLN("VBat is low, goto sleep....");

		WDOG_Feed();

		deep_sleep();
	}

	#if 0
	if (need_sleep == true || 0 == digitalRead(KEY_PIN)
		|| vbat_low == true) {

		INFOLN("deep sleep..");

		WDOG_Feed();

		deep_sleep();

		EMU_EnterEM2(true);

		if (need_sleep == false && vbat_low == false) {
			power_on_dev();
			setup_lora();
		}
	}
	#endif
}
