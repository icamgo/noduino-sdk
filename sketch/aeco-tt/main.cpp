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

extern "C"{
#include "timex.h"
}

#include "rtcdriver.h"
#include "math.h"
#include "em_wdog.h"

#define	DEBUG					1

/* Timer used for bringing the system back to EM0. */
RTCDRV_TimerID_t xTimerForWakeUp;

static uint32_t sample_period = 120;		/* 240s */

static uint32_t sample_count = 0;
#define		HEARTBEAT_TIME			7200

static float old_temp = 0.0;
static float cur_temp = 0.0;

#define	TX_TESTING				1

#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define MODEM_ON_PIN			2		/* PIN6_PB8_D2 */
#define MODEM_RESET_PIN			1		/* PIN5_PB7_D1 */

#define	KEY_PIN					0		/* PIN01_PA00_D0 */

static uint8_t need_push = 0;

uint16_t tx_count = 0;

#define	MQTT_MSG		"{\"ts\":%d`\"gid\":\"%s\"`\"B\":%s`\"T\":%s`\"tp\":%d}"

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

int tx_cause = RESET_TX;

M5311 modem;

char dev_id[24];
char dev_vbat[6];
char dev_data[8];

void push_data();

uint64_t get_devid()
{
	uint64_t *p;

	p = (uint64_t *)0x0FE00008;

	return *p;
}

char *decode_devid(uint64_t n)
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

char *decode_vbat(float vb)
{
	ftoa(dev_vbat, vb, 1);
	return dev_vbat;
}

char *decode_sensor_data(float dd)
{
	ftoa(dev_data, dd, 1);
	return dev_data;
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
	delay(200);
}

void power_off_modem()
{
	digitalWrite(MODEM_ON_PIN, HIGH);
	delay(8300);
	digitalWrite(MODEM_ON_PIN, LOW);

	digitalWrite(PWR_CTRL_PIN, LOW);
}

void wakeup_modem()
{
	digitalWrite(MODEM_ON_PIN, HIGH);
	delay(100);
	digitalWrite(MODEM_ON_PIN, LOW);
	delay(200);
}

void reset_modem()
{
	digitalWrite(MODEM_RESET_PIN, HIGH);
	delay(300);
	digitalWrite(MODEM_RESET_PIN, LOW);
	delay(200);
}

void check_sensor(RTCDRV_TimerID_t id, void *user)
{
	fix_seconds(sample_period);

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

	//power_off_dev();

#ifdef TX_TESTING
	tx_cause = TIMER_TX;
	need_push = 0x5a;
#else
	if (fabsf(cur_temp - old_temp) > 1.0) {

		tx_cause = KEY_TX;
		need_push = 0x5a;
	}
#endif
}

void trig_check_sensor()
{
	noInterrupts();

	need_push = 0x5a;
	tx_cause = KEY_TX;

	interrupts();
}

bool qsetup()
{
	bool network_ok = false;
	int start_cnt = 0;

	Serial1.setRouteLoc(0);
	Serial1.begin(115200);

	//Serial1.println("Hello");
	modem.init(Serial1);

qsetup_start:

	power_on_dev();		// turn on device power
	power_on_modem();

	modem.init_modem();

	WDOG_Feed();

	int ret;

#if 1
	ret = modem.check_boot();
	start_cnt++;

	if (ret == 1) {

		network_ok = true;

	} else if (ret == 2 && start_cnt < 3) {

		power_off_dev();
		delay(1000);
		goto qsetup_start;
	}
#endif

	for (int i = 0; i < 30; i++) {

		WDOG_Feed();
		ret = modem.check_network();

		if (ret == 1) {
			network_ok = true;
			break;

		} else if (ret == 2){

			INFOLN("Try to wakeup modem");
			wakeup_modem();
		}

		delay(1000);

		INFOLN("network check");

	#if 0
		power_off_dev();
		delay(1000);
		power_on_dev();

		power_on_modem();
	#else
		//reset_modem();
	#endif

		//modem.init_modem();
	}

	if (network_ok) {

		//modem.disable_deepsleep();

		//INFOLN("IMEI = " + modem.get_imei());
		//INFOLN("IMSI = " + modem.get_imsi());

		//INFOLN(modem.check_ipaddr());

		char ntm[32];
		memset(ntm, 0, 32);

		modem.get_net_time().toCharArray(ntm, 32);

		INFOLN(ntm);

		uint32_t sec = str2seconds(ntm);

		INFO("epoch = ");
		INFOLN(sec);

	} else {
		/* attach network timeout */
		power_on_modem();
		delay(100);
		modem.clean_net_cache();
	}

	return network_ok;
}

char strtest[] = "+CCLK: 21/02/26,06:22:38+32";

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
	pinMode(MODEM_RESET_PIN, OUTPUT);

	power_off_dev();

	digitalWrite(MODEM_ON_PIN, LOW);
	digitalWrite(MODEM_RESET_PIN, LOW);

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
	//if (0 == digitalRead(KEY_PIN)) {
		/* storage mode */
		tx_cause = RESET_TX;
		need_push = 0x5a;
	//}

	INFOLN("\r\n\r\nAECO-TT setup OK");

	uint32_t sec = str2seconds(strtest);
	INFO("test = ");
	INFOLN(sec);
}


void push_data()
{
	long start_send;
	long end_send;

	uint8_t r_size;

	int e = 0;

	start_send = millis();

	pt1000_init();
	cur_temp = pt1000_get_temp();		// 'C

	float vbat = adc.readVbat();

	if (qsetup()) {

		modem.mqtt_end();
		delay(100);

		char *devid = decode_devid(get_devid());

		wakeup_modem();
		modem.mqtt_begin("mqtt.autoeco.net", 1883, devid);

		char msg[128];
		memset(msg, 0, 128);
		sprintf(msg, MQTT_MSG,
				seconds(),
				devid,
				decode_vbat(vbat),
				decode_sensor_data(cur_temp),
				tx_cause
		);

		wakeup_modem();

#if 1
		if (modem.mqtt_connect()) {

			wakeup_modem();

			if (modem.mqtt_pub("dev/gw", msg)) {

				old_temp = cur_temp;
			}
		}
#else
		modem.mqtt_connect();
		wakeup_modem();
		modem.mqtt_pub("dev/gw", msg);
#endif
		modem.mqtt_end();

	} else {

		// connect basestation failed
		WDOG_Feed();
		//power_off_modem();
		power_off_dev();
	}

	end_send = millis();
}

void loop()
{
	if (0x5a == need_push) {
		INFO("Seconds: ");
		INFOLN(seconds());

		push_data();
		need_push = 0;
	}

	//power_off_dev();
	/*
	 * Enable rtc timer before enter deep sleep
	 * Stop rtc timer after enter check_sensor_data()
	 */
	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypeOneshot, sample_period * 1000, check_sensor, NULL);

	EMU_EnterEM2(true);
}
