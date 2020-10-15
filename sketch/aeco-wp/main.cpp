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
#include "softi2c.h"
#include "pc10.h"

#include "rtcdriver.h"
#include "math.h"
#include "em_wdog.h"

//#define	DEBUG					1

//#define CONFIG_2MIN				1

#define ENABLE_P_TEST			1

#define FW_VER					"Ver 1.3"

#ifdef ENABLE_P_TEST
#define DELTA_P					0.1
#else
#define DELTA_P					0.05
#endif

#define ENABLE_CAD				1

#define ENABLE_CRYPTO				1
#define	PAYLOAD_LEN					30		/* 26+2+4 = 32B */

#ifdef ENABLE_CRYPTO
#include "crypto.h"
#endif

#ifdef ENABLE_P_TEST
static uint32_t cnt_01 = 0;
#endif

/* Timer used for bringing the system back to EM0. */
RTCDRV_TimerID_t xTimerForWakeUp;

#ifndef CONFIG_2MIN
static uint32_t sample_period = 18;		/* 20s */
static uint32_t cnt_20s = 0;
#define HEARTBEAT_TIME			6600	/* 120min */

#else
static uint32_t sample_period = 110;	/* 120s */
//static uint32_t sample_period = 18;	/* 20s */
#endif

static float old_water_h = 0.0;
static float cur_water_h = 0.0;

static float cur_curr = 0.0;

static uint8_t need_push = 0;

#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define	KEY_PIN					0		/* PIN01_PA00_D0 */

#define SDA_PIN					12		/* PIN23_PE12 */
#define SCL_PIN					13		/* PIN24_PE13 */

#define	TX_TIME					1800		// 1800ms
#define DEST_ADDR				1

#ifdef CONFIG_V0
#define TXRX_CH				CH_01_472
#define LORA_MODE			12

#define	RESET_TX			0
#define	DELTA_TX			1
#define	TIMER_TX			2
#define	KEY_TX				3
#define	EL_TX				4
#define	WL_TX				5

#else
#define node_addr				110

#define TXRX_CH				CH_00_470
#define LORA_MODE			11
#endif

#define MAX_DBM					20


#ifdef CONFIG_V0
uint8_t message[PAYLOAD_LEN+6] __attribute__((aligned(4)));
int tx_cause = RESET_TX;
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
	for(int i=0; i<3; i++) {
		temp += adc.temperatureCelsius();
	}
	temp /= 3.0;

	return temp;
}

float fetch_current()
{
	adc.reference(adcRef1V25);

	int ad = 0;

	for (int i = 0; i < 5; i++) {
		ad += adc.read(A6, A7);
	}

	cur_curr = 1250.0*ad/2.0/2048.0/0.7 / 5;

	INFO("ADC differential ch6 ch7 read:");
	INFOLN(ad);

	INFO("The consumption current (mA): ");

	return cur_curr;
}

void check_sensor(RTCDRV_TimerID_t id, void *user)
{
	(void)id;
	(void)user;

	WDOG_Feed();

	RTCDRV_StopTimer(xTimerForWakeUp);

#ifndef CONFIG_2MIN
	cnt_20s++;

	if (cnt_20s >= HEARTBEAT_TIME/20) {

		// 2 hours

		need_push = 0x5a;
		tx_cause = TIMER_TX;
		cnt_20s = 0;
	}
#endif

	pressure_init(SCL_PIN, SDA_PIN);	// initialization of the sensor

	/*
	 * Return value:
	 *
	 *   unit is 'm'
	 *
	 *   183.673469 m: Max valid value (16383)
	 *  -1.53061224 m: Min valid value (1501)
	 *  -1 m: Bus error, no sensor connected, not wakeup
	 *  -2 m: Out of low range
	 *  -3 m: Out of high range
	*/
	cur_water_h = get_water_h();

#ifndef CONFIG_2MIN
	//if (fabsf(cur_water_h - old_water_h) > 0.05) {
	/* 1/200 or 1/100 */
	float dp = fabsf(cur_water_h - old_water_h);

	if (dp >= DELTA_P) {

		need_push = 0x5a;
		tx_cause = DELTA_TX;

	#ifdef ENABLE_P_TEST
		cnt_01 = 0;
	#endif

		return;
	}

	#ifdef ENABLE_P_TEST
	if (dp >= DELTA_P/2 && dp < DELTA_P) {

		cnt_01++;

		if (cnt_01 >= 3) {
			need_push = 0x5a;
			tx_cause = DELTA_TX;

			cnt_01 = 0;
		}

	} else if (dp < DELTA_P/2) {

		cnt_01 = 0;
	}
	#endif

#else

	// 2min fixed interval
	need_push = 0x5a;
	tx_cause = TIMER_TX;

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

#ifdef ENABLE_CRYPTO
	crypto_init();
#endif

#ifndef CONFIG_2MIN
	wInit.perSel = wdogPeriod_32k;	/* 32k 1kHz periods should give 32 seconds */
#else
	wInit.perSel = wdogPeriod_128k;	/* 128 seconds watchdog */
#endif

	// init dev power ctrl pin
	pinMode(PWR_CTRL_PIN, OUTPUT);
	power_off_dev();

	pinMode(KEY_PIN, INPUT);
	attachInterrupt(KEY_PIN, trig_check_sensor, FALLING);

	/* Initialize RTC timer. */
	RTCDRV_Init();
	RTCDRV_AllocateTimer(&xTimerForWakeUp);

#ifdef DEBUG
	Serial.setRouteLoc(1);
	Serial.begin(9600);
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

	int e;

#ifdef CONFIG_V0
	uint8_t *pkt = message;

	memset(pkt, 0, PAYLOAD_LEN+6);
	pkt[0] = 0x47; pkt[1] = 0x4F; pkt[2] = 0x33;
#endif

	////////////////////////////////
	cur_curr = fetch_current();

	noInterrupts();

	if (cur_curr > 1.9)
		pkt[15] = EL_TX;
	else
		pkt[15] = tx_cause;

	interrupts();
	////////////////////////////////

	vbat = adc.readVbat();

	power_on_dev();		// turn on device power

	if (KEY_TX == tx_cause || RESET_TX == tx_cause) {
		pressure_init(SCL_PIN, SDA_PIN);
		cur_water_h = get_water_h();		// water level using cm unit
	}

#ifdef CONFIG_V0
	// set devid
	uint64_t devid = get_devid();
	uint8_t *p = (uint8_t *) &devid;

	for(int i = 0; i < 8; i++) {
		pkt[3+i] = p[7-i];
	}

	/* water_h is m, x100 is cm */
	int16_t ui16 = (int16_t)(cur_water_h * 100);
	p = (uint8_t *) &ui16;

	pkt[11] = p[1]; pkt[12] = p[0];

	ui16 = vbat * 1000;
	pkt[13] = p[1]; pkt[14] = p[0];

	float chip_temp = fetch_mcu_temp();

	// Humidity Sensor data	or Water Leak Sensor data
	pkt[20] = 0;

	// Internal Temperature of the chip
	pkt[21] = (int8_t)roundf(chip_temp);

	// Internal humidity to detect water leak of the shell
	pkt[22] = 255;

	// Internal current consumption
	pkt[23] = (int8_t)roundf(cur_curr);

	p = (uint8_t *) &tx_count;
	pkt[PAYLOAD_LEN-2] = p[1]; pkt[PAYLOAD_LEN-1] = p[0];
	tx_count++;

	/////////////////////////////////////////////////////////
	/*
	 * 2. crc
	 * 3. set mic
	*/

	ui16 = get_crc(pkt, PAYLOAD_LEN);
	p = (uint8_t *) &ui16;
	pkt[PAYLOAD_LEN] = p[1]; pkt[PAYLOAD_LEN+1] = p[0];

#ifdef ENABLE_CRYPTO
	set_pkt_mic(pkt, PAYLOAD_LEN+6);
#endif
	/////////////////////////////////////////////////////////
#else
	uint8_t r_size;

	char vbat_s[10], pres_s[10];
	ftoa(vbat_s, vbat, 2);
	ftoa(pres_s, cur_water_h, 2);

	r_size = sprintf((char *)message, "\\!U/%s/P/%s", vbat_s, pres_s);

	INFO("Sending ");
	INFOLN((char *)message);

	INFO("Real payload size is ");
	INFOLN(r_size);
#endif

	qsetup();

#ifdef ENABLE_CAD
	sx1272.CarrierSense();
#endif

#ifdef DEBUG
	startSend = millis();
#endif

#ifdef CONFIG_V0
	e = sx1272.sendPacketTimeout(DEST_ADDR, message, PAYLOAD_LEN+6, TX_TIME);
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

	INFO("LoRa pkt size ");
	INFOLN(r_size);
#endif

#ifndef CONFIG_2MIN
	if (0 == e) {
		/* send message succesful, update the old_water_h */
		old_water_h = cur_water_h;
	}
#endif

#ifdef DEBUG
	endSend = millis();

	INFO("LoRa Sent in ");
	INFOLN(endSend - startSend);

	INFO("LoRa Sent w/CAD in ");
	INFOLN(endSend - sx1272._startDoCad);

	INFO("Packet sent, state ");
	INFOLN(e);
#endif

	e = sx1272.setSleepMode();

#ifdef DEBUG
	if (!e)
		INFO("Successfully switch into sleep mode");
	else
		INFO("Could not switch into sleep mode");
#endif

	digitalWrite(SX1272_RST, LOW);

	spi_end();

	power_off_dev();
}

void loop()
{

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

	wire_end();

	EMU_EnterEM2(true);
}
