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

#include "softspi.h"
#include "sx1272.h"
#include "pt1000.h"

#include "rtcdriver.h"
#include "math.h"
#include "em_wdog.h"


//#define	DEBUG					1

/* Timer used for bringing the system back to EM0. */
RTCDRV_TimerID_t xTimerForWakeUp;

static uint32_t sample_period = 60;		/* 20s */

static uint32_t sample_count = 0;
#define		HEARTBEAT_TIME			7200

static float old_temp = 0.0;
static float cur_temp = 0.0;

//#define	TX_TESTING				1

#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define	KEY_PIN					0		/* PIN01_PA00_D0 */
#define	WATER_LEAK_PIN			16		/* PIN21_PF02_D16 */

static uint8_t need_push = 0;

#define ENABLE_CAD				1

#define	TX_TIME					1800		// 1800ms
#define DEST_ADDR				1

#ifdef CONFIG_V0
#define TXRX_CH				CH_01_472
#define LORA_MODE			12

#define	RESET_TX			0
#define	DELTA_TX			1
#define	TIMER_TX			2
#define	KEY_TX				3
#define	WATER_LEAK_TX		4

#else
#define node_addr				110

#define TXRX_CH				CH_00_470
#define LORA_MODE			11
#endif

#define MAX_DBM					20


//#define WITH_ACK

#ifdef CONFIG_V0
uint8_t message[32] = { 0x47, 0x4F, 0x33 };
uint8_t tx_cause = RESET_TX;
uint16_t tx_count = 0;
#else
uint8_t message[32];
#endif

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

#ifdef WITH_ACK
#define	NB_RETRIES			2
#endif

void push_data(bool al);

char *ftoa(char *a, double f, int precision)
{
	long p[] =
	    { 0, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000 };

	char *ret = a;
	long heiltal = (long)f;
	itoa(heiltal, a, 10);
	while (*a != '\0')
		a++;
	*a++ = '.';
	long desimal = abs((long)((f - heiltal) * p[precision]));
	if (desimal < p[precision - 1]) {
		*a++ = '0';
	}
	itoa(desimal, a, 10);
	return ret;
}

void power_on_dev()
{
	digitalWrite(PWR_CTRL_PIN, HIGH);
}

void power_off_dev()
{
	digitalWrite(PWR_CTRL_PIN, LOW);
}

void check_sensor(RTCDRV_TimerID_t id, void *user)
{
	(void)id;
	(void)user;

	WDOG_Feed();

	RTCDRV_StopTimer(xTimerForWakeUp);

	sample_count++;

	if (sample_count >= HEARTBEAT_TIME/20) {
		need_push = 0x5a;
		tx_cause = TIMER_TX;
		sample_count = 0;
	}

	power_on_dev();		// turn on device power

	pt1000_init();	// initialization of the sensor

	cur_temp = pt1000_get_temp();

	power_off_dev();

#ifdef TX_TESTING
	need_push = 0x5a;
	tx_cause = TIMER_TX;
#else
	if (fabsf(cur_temp - old_temp) > 1.0) {

		need_push = 0x5a;
#ifdef CONFIG_V0
		tx_cause = DELTA_TX;
#endif
	}
#endif
}

void trig_check_sensor()
{
	noInterrupts();

	need_push = 0x5a;
#ifdef CONFIG_V0
	tx_cause = KEY_TX;
#endif

	interrupts();
}

void water_leak_alarm()
{
	need_push = 0x5a;
#ifdef CONFIG_V0
	tx_cause = WATER_LEAK_TX;
#endif
}

void setup()
{
	Ecode_t e;

#if 0
	/* Initialize EM23 with default parameters */
	EMU_EM23Init_TypeDef em23Init = EMU_EM23INIT_DEFAULT;
	EMU_EM23Init(&em23Init);
#endif

	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = wdogPeriod_32k;	/* 32k 1kHz periods should give 32 seconds */

	// dev power ctrl
	pinMode(PWR_CTRL_PIN, OUTPUT);

	power_off_dev();

	pinMode(KEY_PIN, INPUT);
	attachInterrupt(KEY_PIN, trig_check_sensor, FALLING);

	pinMode(WATER_LEAK_PIN, INPUT);
	attachInterrupt(WATER_LEAK_PIN, water_leak_alarm, FALLING);

	/* Initialize RTC timer. */
	RTCDRV_Init();
	RTCDRV_AllocateTimer(&xTimerForWakeUp);

#ifdef DEBUG
	Serial.setRouteLoc(1);
	Serial.begin(115200);
#endif

	/* Start watchdog */
	WDOG_Init(&wInit);

	/* bootup tx */
	tx_cause = RESET_TX;
	need_push = 0x5a;
}

void qsetup()
{
	power_on_dev();		// turn on device power

#ifdef CONFIG_V0
	sx1272.setup_v0(TXRX_CH, MAX_DBM);
#else
	sx1272.sx1278_qsetup(TXRX_CH, MAX_DBM);
	sx1272._nodeAddress = node_addr;
#endif

#ifdef ENABLE_CAD
	sx1272._enableCarrierSense = true;
#endif

}

#ifdef CONFIG_V0
uint64_t get_devid()
{
#if 1
	uint64_t *p;

	p = (uint64_t *)0x0FE00008;

	return *p;
#else
	return 11912510001ULL;		/* T2w */
#endif
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

void push_data(bool alarm)
{
	long startSend;
	long endSend;

	float vbat = 0.0;

	uint8_t r_size;

	int e;

	qsetup();

	if (KEY_TX == tx_cause || RESET_TX == tx_cause ||
		WATER_LEAK_TX == tx_cause) {
		pt1000_init();
		cur_temp = pt1000_get_temp();		// 'C
	}

	if (WATER_LEAK_TX == tx_cause || 0 == digitalRead(WATER_LEAK_PIN)) {
		cur_temp += 300.0;
	}

	vbat = adc.readVbat();

#ifdef CONFIG_V0
	uint8_t *pkt = message;
	uint64_t devid = get_devid();

	uint8_t *p = (uint8_t *) &devid;

	// set devid
	int i = 0;
	for(i = 0; i < 8; i++) {
		pkt[3+i] = p[7-i];
	}

	int16_t ui16 = (int16_t)(cur_temp * 10);
	p = (uint8_t *) &ui16;

	pkt[11] = p[1]; pkt[12] = p[0];

	ui16 = vbat * 1000;
	pkt[13] = p[1]; pkt[14] = p[0];

	pkt[15] = tx_cause;

	p = (uint8_t *) &tx_count;
	pkt[16] = p[1]; pkt[17] = p[0];
	tx_count++;

	ui16 = get_crc(pkt, 18);
	p = (uint8_t *) &ui16;
	pkt[18] = p[1]; pkt[19] = p[0];

#else
	char vbat_s[10], pres_s[10];
	ftoa(vbat_s, vbat, 2);
	ftoa(pres_s, cur_temp, 2);

	r_size = sprintf((char *)message, "\\!U/%s/P/%s", vbat_s, pres_s);

	INFO("Sending ");
	INFOLN((char *)message);

	INFO("Real payload size is ");
	INFOLN(r_size);
#endif

#ifdef ENABLE_CAD
	sx1272.CarrierSense();
#endif

	startSend = millis();

#ifdef CONFIG_V0
	e = sx1272.sendPacketTimeout(DEST_ADDR, message, 24, TX_TIME);
#else
	// just a simple data packet
	sx1272.setPacketType(PKT_TYPE_DATA);

	// Send message to the gateway and print the result
	// with the app key if this feature is enabled
#ifdef WITH_ACK
	int n_retry = NB_RETRIES;

	do {
		e = sx1272.sendPacketTimeoutACK(DEST_ADDR,
						message, r_size);

		if (e == 3)
			INFO("No ACK");

		n_retry--;

		if (n_retry)
			INFO("Retry");
		else
			INFO("Abort");

	} while (e && n_retry);
#else
	// 10ms max tx time
	e = sx1272.sendPacketTimeout(DEST_ADDR, message, r_size, TX_TIME);
#endif
#endif

	if (!e) {
		// send message succesful, update the old_temp
		old_temp = cur_temp;
	}

	endSend = millis();

	INFO("LoRa pkt size ");
	INFOLN(r_size);

	INFO("LoRa Sent in ");
	INFOLN(endSend - startSend);

	INFO("LoRa Sent w/CAD in ");
	INFOLN(endSend - sx1272._startDoCad);

	INFO("Packet sent, state ");
	INFOLN(e);

	e = sx1272.setSleepMode();
	if (!e)
		INFO("Successfully switch into sleep mode");
	else
		INFO("Could not switch into sleep mode");

	digitalWrite(SX1272_RST, LOW);

	spi_end();

	power_off_dev();
}

void loop()
{
	INFO("Clock Freq = ");
	INFOLN(CMU_ClockFreqGet(cmuClock_CORE));

	//INFOLN("Feed the watchdog");

	if (0x5a == need_push) {
		push_data(false);

		need_push = 0;
	}

	power_off_dev();
	digitalWrite(SX1272_RST, LOW);

	spi_end();

	/*
	 * Enable rtc timer before enter deep sleep
	 * Stop rtc timer after enter check_sensor_data()
	 */
	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypeOneshot, sample_period * 1000, check_sensor, NULL);

	EMU_EnterEM2(true);
}
