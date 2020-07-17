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

#include "rtcdriver.h"
#include "math.h"
#include "em_wdog.h"

//#define	DEBUG					1

/* Timer used for bringing the system back to EM0. */
RTCDRV_TimerID_t xTimerForWakeUp;

static uint32_t sample_period = 30;		/* 30s */

static uint32_t sample_count = 0;
#define		HEARTBEAT_TIME			7200

static float old_temp = 0.0;
static float cur_temp = 0.0;

uint8_t wf_pkt3[24] = {
	0x47, 0x4F, 0x31,
	0x83, 0x31, 0x43, 0xC4, 0x2F, 0x86, 0x09, 0x2B,
	0xA2, 0x53, 0xF8, 0x47, 0x0E, 0x2A,	/* 0x05 9F */
	0x8B, 0x06, 0x62, 0xF3, 0x97, 0x4A, 0xE7
};

uint8_t wf_pkt2[28] = {
	0x47, 0x4F, 0x31,
	0x00, 0x00, 0x01, 0x02, 0x22, 0x33, 0x44, 0x55,
	0x22, 0x33, 0x44, 0x55, 0x02, 0xA6,
	0x8B, 0x03, 0xd9, 0x00, 0x00, 0x04, 0xb5,
	0x00, 0x00, 0x00, 0x00
};

uint8_t wf_pkt1[24] = {
	0x47, 0x4F, 0x31,
	0x83, 0x31, 0x43, 0xCF, 0x27, 0x8B, 0x01, 0x2B,
	0xC2, 0x33, 0xF8, 0x47, 0x05, 0x9F, /* 0x05 9F */
	0x8B, 0x06, 0x62, 0x99, 0xF2, 0xAE, 0x55
};

//#define	TX_TESTING				1

#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define	KEY_PIN					0		/* PIN01_PA00_D0 */

static uint8_t need_push = 0;

#define ENABLE_CAD				1

#define	TX_TIME					2300		// 2300ms
#define DEST_ADDR				1

#ifdef CONFIG_V0
#define TXRX_CH				CH_01_472
#define LORA_MODE			12

#define	RESET_TX			0
#define	DELTA_TX			1
#define	TIMER_TX			2
#define	KEY_TX				3

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

void push_data();

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

float fetch_mcu_temp()
{
	float temp = 0.0;
	for(int i=0; i<5; i++) {
		temp += adc.temperatureCelsius();
	}
	temp /= 5.0;

	return temp;
}

void check_sensor(RTCDRV_TimerID_t id, void *user)
{
	(void)id;
	(void)user;

	WDOG_Feed();

	RTCDRV_StopTimer(xTimerForWakeUp);

#ifdef TX_TESTING
	need_push = 0x5a;
	tx_cause = TIMER_TX;
#else
	sample_count++;

	if (sample_count >= HEARTBEAT_TIME/20) {
		need_push = 0x5a;
		tx_cause = TIMER_TX;
		sample_count = 0;
	}

	cur_temp = fetch_mcu_temp();

	if (fabsf(cur_temp - old_temp) > 0.5) {

		need_push = 0x5a;
#ifdef CONFIG_V0
		tx_cause = DELTA_TX;
#endif
	}
#endif
}

void trig_check_sensor()
{
	need_push = 0x5a;
#ifdef CONFIG_V0
	tx_cause = KEY_TX;
#endif
}

void setup()
{
	Ecode_t e;

	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = wdogPeriod_64k;	/* 128k 1kHz periods should give 64 seconds */

	// dev power ctrl
	pinMode(PWR_CTRL_PIN, OUTPUT);

	power_off_dev();

	pinMode(KEY_PIN, INPUT);
	attachInterrupt(KEY_PIN, trig_check_sensor, FALLING);

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
	uint64_t *p;

	p = (uint64_t *)0x0FE00008;

	return *p;
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

void push_data()
{
	long startSend;
	long endSend;

	float vbat = 0.0;

	uint8_t r_size;

	int e;

	vbat = adc.readVbat();

#ifdef TX_TESTING
	cur_temp = fetch_mcu_temp();
#else
	if (KEY_TX == tx_cause || RESET_TX == tx_cause) {
		cur_temp = fetch_mcu_temp();
	}
#endif

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

	pkt[20] = 0;		// Humidity Sensor data	or Water Leak Sensor data
	pkt[21] = 0;		// Internal Temperature of the chip
	pkt[22] = 0;		// Internal humidity to detect water leak of the shell
	pkt[23] = 0;		// Internal current consumption
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

	power_on_dev();
	qsetup();

#ifdef ENABLE_CAD
	sx1272.CarrierSense();
#endif

#ifdef DEBUG
	startSend = millis();
#endif

#ifdef CONFIG_V0
	if (tx_cause != KEY_TX) {
		e = sx1272.sendPacketTimeout(DEST_ADDR, message, 24, TX_TIME);
	} else {
		e = sx1272.sendPacketTimeout(DEST_ADDR, wf_pkt2, 28, TX_TIME);
	}
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

#ifdef DEBUG
	endSend = millis();

	INFO("LoRa pkt size ");
	INFOLN(r_size);

	INFO("LoRa Sent in ");
	INFOLN(endSend - startSend);

	INFO("LoRa Sent w/CAD in ");
	INFOLN(endSend - sx1272._startDoCad);

	INFO("Packet sent, state ");
	INFOLN(e);
#endif

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
		push_data();

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
