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

#define DEBUG							1
#define ENABLE_RTC						1
//#define DEBUG_RTC						1

#define	FW_VER						"V1.7"

#ifdef ENABLE_RTC
#include "softi2c.h"
#include "pcf8563.h"
#endif

/* Timer used for bringing the system back to EM0. */
RTCDRV_TimerID_t xTimerForWakeUp;

static uint32_t check_period = 18;		/* 20s = 0.33min */
static uint32_t sample_period = 90;		/* 20s * 90 = 1800s, 30min */

#define WDOG_PERIOD				wdogPeriod_32k			/* 32k 1kHz periods should give 32 seconds */

#ifdef EFM32ZG110F32
#define SAMPLE_PERIOD			90		/* check_period x SAMPLE_PERIOD = 1800s (30min) */
#define PUSH_PERIOD				720		/* check_period x PUSH_PERIOD = 240min, 4h */
#elif EFM32HG110F64
#define SAMPLE_PERIOD			90		/* check_period x SAMPLE_PERIOD = 1800s (30min) */
#define PUSH_PERIOD				1080	/* check_period x PUSH_PERIOD = 360min, 6h */
#define	MQTT_INIT_MSG			"{\"ts\":%d`\"gid\":\"%s\"`\"B\":%s`\"T\":%s`\"iT\":%d`\"tp\":%d`\"L\":%d`\"sid\":\"%s\"}"
#endif

#define INIT_TS						1614665566UL
#define MAX_TS						2000000000UL

static uint32_t sample_count = 0;

static float cur_temp = 0.0;
static int cur_water = 0.0;

static float old_temp = 0.0;
static int old_water = 0.0;

#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define MODEM_ON_PIN			2		/* PIN6_PB8_D2 */
#define MODEM_RESET_PIN			1		/* PIN5_PB7_D1 */

#define	KEY_PIN					0		/* PIN01_PA00_D0 */

#define	RTC_INT_PIN				16		/* PIN21_PF02_D16 */

#define SDA_PIN					12		/* PIN23_PE12 */
#define SCL_PIN					13		/* PIN24_PE13 */

static uint8_t need_push = 0;

uint16_t tx_count = 0;

#define	MQTT_MSG		"{\"ts\":%d`\"gid\":\"%s\"`\"B\":%s`\"T\":%s`\"iT\":%d`\"tp\":%d`\"L\":%d`\"sgi\":%d}"

#define LEVEL_UNKNOWN				-2
#define LEVEL_LOW					-1
#define LEVEL_MEDIAN				0
#define LEVEL_HIGH					1

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

#ifdef EFM32HG110F64
char iccid[24] __attribute__((aligned(4)));
int g_rssi = 0;
#endif

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

void check_sensor(RTCDRV_TimerID_t id, void *user)
{
	int ret = 0;

	WDOG_Feed();

	RTCDRV_StopTimer(xTimerForWakeUp);

	fix_seconds(check_period + check_period/9);

	//INFOLN(seconds());

	sample_count++;

	if (sample_count % sample_period == 0) {
		/* 20s x 60 = 1200s, 20min, sample a point */
		power_on_dev();
		cur_temp = get_temp();
		cur_water = get_water();
		power_off_dev();

		#ifdef ENABLE_RTC
		pcf8563_init(SCL_PIN, SDA_PIN);
		ret = push_point(&g_cbuf, pcf8563_now(), (cur_temp * 10), fetch_mcu_temp(), cur_water);
		#else
		ret = push_point(&g_cbuf, seconds(), (cur_temp * 10), fetch_mcu_temp(), cur_water);
		#endif

		if (ret == 0) {
			/*
			 * point is saved ok
			 * reset sample period to 20min
			*/
			sample_period = 90;

		} else if (ret == 1) {
			/*
			 * cbuf is full
			 * change the sample period to 60min
			*/
			sample_period = 180;
		}

		if ((cur_water != old_water) || fabsf(cur_temp - old_temp) > 5.0) {
			/* Level is changed, need to push */
			need_push = 0x5a;
			tx_cause = DELTA_TX;
		}

		old_temp = cur_temp;
		old_water = cur_water;
	}

	if (sample_count >= PUSH_PERIOD) {

		/* 4min * 15 =  60min */
		need_push = 0x5a;
		tx_cause = TIMER_TX;
		sample_count = 0;
	}
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

	#ifdef EFM32HG110F64
	memset(iccid, 0, 24);
	modem.get_iccid().toCharArray(iccid, 24);
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

	power_on_dev();
	cur_temp = get_temp();
	cur_water = get_water();
	power_off_dev();

#ifdef ENABLE_RTCx
	push_point(&g_cbuf, pcf8563_now(), (cur_temp * 10), fetch_mcu_temp(), cur_water);
#else
	push_point(&g_cbuf, seconds(), (cur_temp * 10), fetch_mcu_temp(), cur_water);
#endif
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

			while (get_1st_point(&g_cbuf, &d) == 0 && cnt_fail < 15) {

				WDOG_Feed();
				wakeup_modem();

				if (d.ts < INIT_TS || d.ts > MAX_TS) {
					INFOLN("Invalid ts, use current ts");
					d.ts = seconds();
					INFOLN(seconds());
				}

				extern char modem_said[MODEM_LEN];
				memset(modem_said, 0, MODEM_LEN);

				#ifdef EFM32HG110F64
				if (tx_cause == 0 || gmtime(&(d.ts))->tm_hour == 12) {

					sprintf(modem_said, MQTT_INIT_MSG,
							d.ts,
							devid,
							decode_vbat(vbat),
							decode_sensor_data(d.data / 10.0),
							d.iT,
							tx_cause,
							d.wl,
							iccid
					);
				} else {
				#endif
				sprintf(modem_said, MQTT_MSG,
						d.ts,
						devid,
						decode_vbat(vbat),
						decode_sensor_data(d.data / 10.0),
						d.iT,
						tx_cause,
						d.wl,
						g_rssi
				);
				#ifdef EFM32HG110F64
				}
				#endif

				if (modem.mqtt_pub("dev/gw", modem_said)) {

					INFOLN("Pub OK, pop point");
					noInterrupts();
					pop_point(&g_cbuf, &d);
					interrupts();

				} else {

					cnt_fail++;
					INFOLN("Pub failed");
				}
			}
			INFOLN("no point");
		}

		WDOG_Feed();
		modem.mqtt_end();
	}

	WDOG_Feed();

	power_off_modem();
	power_off_dev();
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

	//wire_end();
	//digitalWrite(SCL_PIN, HIGH);
	//digitalWrite(SDA_PIN, HIGH);

	power_off_dev();
	/*
	 * Enable rtc timer before enter deep sleep
	 * Stop rtc timer after enter check_sensor_data()
	 */
	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypeOneshot, check_period * 1000, check_sensor, NULL);

	EMU_EnterEM2(true);
}
