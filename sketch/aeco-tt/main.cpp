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

#include "pt1000.h"
#include "m5311.h"

#include "rtcdriver.h"
#include "math.h"
#include "em_wdog.h"

//#define	DEBUG					1

/* Timer used for bringing the system back to EM0. */
RTCDRV_TimerID_t xTimerForWakeUp;

static uint32_t sample_period = 240;		/* 240s */

static uint32_t sample_count = 0;
#define		HEARTBEAT_TIME			7200

static float old_temp = 0.0;
static float cur_temp = 0.0;

//#define	TX_TESTING				1

#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define MODEM_ON_PIN			2		/* PIN6_PB8_D2 */

#define	KEY_PIN					0		/* PIN01_PA00_D0 */

static uint8_t need_push = 0;

uint8_t message[32] = { 0x47, 0x4F, 0x33 };
uint16_t tx_count = 0;

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

void push_data();

M5311 modem;

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

void power_on_modem()
{
	digitalWrite(PWR_CTRL_PIN, HIGH);

	digitalWrite(MODEM_ON_PIN, HIGH);
	delay(2000);
	digitalWrite(MODEM_ON_PIN, LOW);
}

void power_off_modem()
{
	digitalWrite(MODEM_ON_PIN, HIGH);
	delay(8300);
	digitalWrite(MODEM_ON_PIN, LOW);

	digitalWrite(PWR_CTRL_PIN, LOW);
}

void check_sensor(RTCDRV_TimerID_t id, void *user)
{
	(void)id;
	(void)user;

	WDOG_Feed();

	RTCDRV_StopTimer(xTimerForWakeUp);

	sample_count++;

#if 0
	if (sample_count >= HEARTBEAT_TIME/20) {
		need_push = 0x5a;
		sample_count = 0;
	}
#endif

	power_on_dev();		// turn on device power

	pt1000_init();	// initialization of the sensor

	cur_temp = pt1000_get_temp();

	power_off_dev();

#ifdef TX_TESTING
	need_push = 0x5a;
#else
	if (fabsf(cur_temp - old_temp) > 1.0) {

		need_push = 0x5a;
	}
#endif
}

void trig_check_sensor()
{
	noInterrupts();

	need_push = 0x5a;

	interrupts();
}

void qsetup()
{
	power_on_dev();		// turn on device power
	power_on_modem();

	Serial1.setRouteLoc(0);
	Serial1.begin(115200);

	//Serial1.println("Hello");

	modem.init(Serial1);

	while (!modem.check_network()) {

		power_off_dev();
		delay(1000);
		power_on_dev();

		power_on_modem();

		modem.init_modem();
	}


	INFOLN("IMEI = " + modem.get_imei());
	INFOLN("IMSI = " + modem.get_imsi());

	delay(500);

	INFOLN(modem.check_ipaddr());

	//Serial1.println(modem.check_ipaddr());
}

void setup()
{
	Ecode_t e;

	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = wdogPeriod_256k;	/* 256k 1kHz periods should give 256 seconds */

	// dev power ctrl
	pinMode(PWR_CTRL_PIN, OUTPUT);

	pinMode(MODEM_ON_PIN, OUTPUT);

	power_off_dev();
	digitalWrite(MODEM_ON_PIN, LOW);

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
	need_push = 0x5a;

	qsetup();
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

	qsetup();

	pt1000_init();
	cur_temp = pt1000_get_temp();		// 'C

	vbat = adc.readVbat();

	startSend = millis();

#ifdef CONFIG_V0
	e = sx1272.sendPacketTimeout(DEST_ADDR, message, 24, TX_TIME);
#endif

	if (!e) {
		// send message succesful, update the old_temp
		old_temp = cur_temp;
	}


	power_off_dev();
}

void loop()
{
	if (0x5a == need_push) {
		push_data();

		need_push = 0;
	}

	power_off_dev();

	/*
	 * Enable rtc timer before enter deep sleep
	 * Stop rtc timer after enter check_sensor_data()
	 */
	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypeOneshot, sample_period * 1000, check_sensor, NULL);

	EMU_EnterEM2(true);
}
