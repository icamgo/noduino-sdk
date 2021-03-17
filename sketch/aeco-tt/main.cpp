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

#include "circ_buf.h"

#define	DEBUG					1

#define	FW_VER						"V1.2"

/* Timer used for bringing the system back to EM0. */
RTCDRV_TimerID_t xTimerForWakeUp;

static uint32_t check_period = 18;		/* 20s = 0.33min */

#define WDOG_PERIOD				wdogPeriod_32k			/* 32k 1kHz periods should give 32 seconds */

#ifdef EFM32ZG110F32
#define SAMPLE_PERIOD			60		/* check_period x SAMPLE_PERIOD = 1200s (20min) */
#define PUSH_PERIOD				360		/* check_period x PUSH_PERIOD = 7200s */
#elif EFM32HG110F64
#define SAMPLE_PERIOD			36		/* check_period x SAMPLE_PERIOD = 720s (12min) */
#define PUSH_PERIOD				360		/* check_period x PUSH_PERIOD = 120min */
#endif

static uint32_t sample_count = 0;

static float old_temp = 0.0;
static float cur_temp = 0.0;

//#define	TX_TESTING				1

#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define MODEM_ON_PIN			2		/* PIN6_PB8_D2 */
#define MODEM_RESET_PIN			1		/* PIN5_PB7_D1 */

#define	KEY_PIN					0		/* PIN01_PA00_D0 */

static uint8_t need_push = 0;

uint16_t tx_count = 0;

#define	MQTT_MSG		"{\"ts\":%d`\"gid\":\"%s\"`\"B\":%s`\"T\":%s`\"iT\":%d`\"tp\":%d}"

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

struct circ_buf g_cbuf;

int tx_cause = RESET_TX;

M5311 modem;

char dev_id[24];
char dev_vbat[6];
char dev_data[8];

//__attribute__((aligned(4)));

void push_data();
int8_t fetch_mcu_temp();

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
	ftoa(dev_vbat, vb, 3);
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
	WDOG_Feed();

	RTCDRV_StopTimer(xTimerForWakeUp);

	fix_seconds(check_period + check_period/9);

	//INFOLN(seconds());

	sample_count++;

#if 1
	if (sample_count % SAMPLE_PERIOD == 0) {

		/* every 4x3 = 12min, sample a point */
		power_on_dev();
		pt1000_init();
		cur_temp = pt1000_get_temp();
		power_off_dev();

		push_point(&g_cbuf, seconds(), (cur_temp * 10), fetch_mcu_temp());
	}

	if (sample_count >= PUSH_PERIOD) {

		/* 4min * 15 =  60min */
		need_push = 0x5a;
		tx_cause = TIMER_TX;
		sample_count = 0;
	}
#else
	power_on_dev();		// turn on device power

	pt1000_init();	// initialization of the sensor

	cur_temp = pt1000_get_temp();

	//power_off_dev();

#ifdef TX_TESTING
	tx_cause = TIMER_TX;
	need_push = 0x5a;
#else
	if (fabsf(cur_temp - old_temp) > 1.0) {

		tx_cause = DELTA_TX;
		need_push = 0x5a;
	}
#endif

#endif
}

void trig_check_sensor()
{
	noInterrupts();

	need_push = 0x5a;
	tx_cause = KEY_TX;

	interrupts();
}

int qsetup()
{
	bool network_ok = false;
	int start_cnt = 0;

	power_on_dev();		// turn on device power

	INFOLN("....");
	SerialUSART1.setRouteLoc(0);
	INFOLN("....");
	SerialUSART1.begin(115200);
	INFOLN("xxxx");

	modem.init(&SerialUSART1);

qsetup_start:

	INFOLN(__LINE__);
	WDOG_Feed();
	INFOLN(__LINE__);
	power_on_modem();
	INFOLN(__LINE__);

#if 0
	int ii;
	for (ii = 0; ii < 5; ii++) {

		if (modem.wait_modem()) {
			modem.init_modem();
			break;
		}

		delay(1000);

		WDOG_Feed();
	}

	if (ii >= 5) {

		// modem serial is not  OK,
		// goto sleep
		return 2;
	}

#else
	delay(1200);
	modem.init_modem();
#endif

	int ret = 0;

	ret = modem.check_boot();
	start_cnt++;

	if (ret == 1) {

		network_ok = true;

	} else if ((ret == 2 || ret == 0) && start_cnt < 3) {
		INFOLN(__LINE__);
		power_off_dev();
		INFOLN(__LINE__);
		delay(1000);
		INFOLN(__LINE__);
		goto qsetup_start;
		INFOLN(__LINE__);
	} else {

		for (int i = 0; i < 20; i++) {

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

	}

	if (network_ok) {

		//modem.disable_deepsleep();

		//INFOLN("IMEI = " + modem.get_imei());
		//INFOLN("IMSI = " + modem.get_imsi());

		//INFOLN(modem.check_ipaddr());

		extern char str[BUF_LEN];
		memset(str, 0, BUF_LEN);

		WDOG_Feed();

		//char strtest[] = "21/02/26,06:22:38+32";
		modem.get_net_time().toCharArray(str, BUF_LEN);

		INFOLN(str);

		uint32_t sec = str2seconds(str);

		if (sec > 1614665568) {
			update_seconds(sec);
		}

		INFO("epoch = ");
		INFOLN(seconds());

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
	INFO("epoch = ");
	INFOLN(seconds());

	power_on_dev();	// turn on device power
	pt1000_init();	// initialization of the sensor
	cur_temp = pt1000_get_temp();
	power_off_dev();

	push_point(&g_cbuf, seconds(), (cur_temp * 10), fetch_mcu_temp());
}

void push_data()
{
	long cnt_fail = 0;

	struct point d;

	WDOG_Feed();

	float vbat = adc.readVbat();

	int ret = get_1st_point(&g_cbuf, &d);

	if (ret == 1) {
		INFOLN("No point in buf");
		return;
	}

	ret = qsetup();
	if (ret == 1) {

		modem.mqtt_end();
		delay(100);

		char *devid = decode_devid(get_devid());

		WDOG_Feed();
		wakeup_modem();
		modem.mqtt_begin("mqtt.autoeco.net", 1883, devid);

		WDOG_Feed();
		wakeup_modem();

		if (modem.mqtt_connect()) {

			WDOG_Feed();

			while (get_1st_point(&g_cbuf, &d) == 0 && cnt_fail < 3) {

				WDOG_Feed();
				wakeup_modem();

				if (d.ts == 0) {
					INFOLN("Do not use ts0, update ts");
					d.ts = seconds();
					INFOLN(seconds());
				}

				extern char modem_said[MODEM_LEN];
				memset(modem_said, 0, MODEM_LEN);
				sprintf(modem_said, MQTT_MSG,
						d.ts,
						devid,
						decode_vbat(vbat),
						decode_sensor_data(d.data / 10.0),
						d.iT,
						tx_cause
				);

				if (modem.mqtt_pub("dev/gw", modem_said)) {

					//old_temp = cur_temp;
					INFOLN("Pub OK, pop point");
					noInterrupts();
					pop_point(&g_cbuf, &d);
					interrupts();

				} else {

					cnt_fail++;
					INFOLN("Pub failed");
				}
			}
		}

		WDOG_Feed();
		modem.mqtt_end();
#if 1
	}

	WDOG_Feed();

	power_off_modem();
	power_off_dev();
#else
	} else if (ret == 2) {
		// modem serial is hung

		WDOG_Feed();
		power_off_modem();
		power_off_dev();

		return;

	} else if (ret == 0) {

		WDOG_Feed();

		modem.enter_deepsleep();

		power_off_modem();
		power_off_dev();
	}
#endif
}

void loop()
{
	if (0x5a == need_push) {
		INFO("Seconds: ");
		INFOLN(seconds());

		push_data();
		need_push = 0;

		WDOG_Feed();
	}

	power_off_dev();
	/*
	 * Enable rtc timer before enter deep sleep
	 * Stop rtc timer after enter check_sensor_data()
	 */
	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypeOneshot, check_period * 1000, check_sensor, NULL);

	EMU_EnterEM2(true);
}
