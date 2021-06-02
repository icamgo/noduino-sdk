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

#include "rtcdriver.h"
#include "math.h"
#include "em_wdog.h"

//#define	DEBUG					1

#define FW_VER						"Ver 1.0"

/* 1 hour - 1 point */
#define CONFIG_2MIN					1

#define ENABLE_CRYPTO				1
#define ENABLE_CAD					1

#define DELTA_T						1.5

#define LOW_BAT_THRESHOLD			2.3
#define	PAYLOAD_LEN					30		/* 30+2+4 = 36B */

#include "sx126x.h"
SX126x sx126x(2,			// Pin: SPI CS,PIN06-PB08-D2
				9,			// Pin: RESET, PIN18-PC15-D9
				5,			// PIN: Busy,  PIN11-PB14-D5, The RX pin
				3			// Pin: DIO1,  PIN08-PB11-D3
);

#ifdef ENABLE_CRYPTO
#include "crypto.h"
#endif

/* Timer used for bringing the system back to EM0. */
RTCDRV_TimerID_t xTimerForWakeUp;

static uint32_t cnt_20s = 0;
static uint32_t check_period = 18;			/* 20s */
static uint32_t sample_period = 180;		/* 180x20 = 3600s */

static uint32_t old_smk = 0;
static uint32_t cur_smk = 0;
static float cur_vbat = 0.0;

bool vbat_low __attribute__((aligned(4))) = false;
int cnt_vbat_low __attribute__((aligned(4))) = 0;
int cnt_vbat_ok __attribute__((aligned(4))) = 0;

static uint32_t need_push = 0;

#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define	KEY_PIN					0		/* PIN01_PA00_D0 */
#define SDA_PIN					12		/* PIN23_PE12 */
#define SCL_PIN					13		/* PIN24_PE13 */
#define	SMK_PIN					SCL_PIN

#define	TX_TIME					1800		// 1800ms

#define TXRX_CH					472500000
#define MAX_DBM					22

#define	RESET_TX			0
#define	DELTA_TX			1
#define	TIMER_TX			2
#define	KEY_TX				3
#define	EL_TX				4
#define	WL_TX				5

uint8_t message[PAYLOAD_LEN+6] __attribute__((aligned(4))) = { 0x47, 0x4F, 0x33 };

uint32_t tx_cause = RESET_TX;
uint16_t tx_count __attribute__((aligned(4))) = 0;
uint32_t tx_ok_cnt = 0;

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

void push_data(bool cad);

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

uint32_t get_smk()
{
	return digitalRead(SMK_PIN);
}

void check_sensor(RTCDRV_TimerID_t id, void *user)
{
	(void)id;
	(void)user;

	WDOG_Feed();

	RTCDRV_StopTimer(xTimerForWakeUp);

	cnt_20s++;

	if (cnt_20s >= sample_period) {
		/* Timer 1, Two hours */
		need_push = 0x5a;
		tx_cause = TIMER_TX;
		cnt_20s = 0;
	}

	////////////////////////////////////////////////////
	// check the low vbat
	cur_vbat = fetch_vbat();
	if (cur_vbat <= LOW_BAT_THRESHOLD) {

		cnt_vbat_low++;

		if (cnt_vbat_low >= 30) {
			/*
			 * vbat is less than 2.8V in 10min
			 * my battery is low
			*/
			cnt_vbat_low = 0;

		}
		cnt_vbat_ok = 0;

	} else {
		cnt_vbat_ok++;

		if (cnt_vbat_ok >= 6) {
			/* vbat is great than 3.5V in 2min */

			vbat_low = false;
			cnt_vbat_ok = 0;

		}
		cnt_vbat_low = 0;
	}

	cur_smk = get_smk();

	if (cur_smk == 0) {
		need_push = 0x5C;
		tx_cause = DELTA_TX;
	}
}

void trig_check_sensor()
{
	noInterrupts();

	need_push = 0x5a;
	tx_cause = KEY_TX;

	interrupts();
}

void smoke_alarm()
{
	noInterrupts();

	need_push = 0x5C;
	tx_cause = EL_TX;

	interrupts();
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

	wInit.perSel = wdogPeriod_32k;	/* 32k 1kHz periods should give 32 seconds */

	cur_vbat = fetch_vbat();
	if (cur_vbat <= 2.2) {
		vbat_low = true;
	}

	// init dev power ctrl pin
	pinMode(PWR_CTRL_PIN, OUTPUT);

	power_off_dev();

	pinMode(KEY_PIN, INPUT);
	attachInterrupt(KEY_PIN, trig_check_sensor, FALLING);

	pinMode(SMK_PIN, INPUT);
	attachInterrupt(SMK_PIN, smoke_alarm, FALLING);

	/* Initialize RTC timer. */
	RTCDRV_Init();
	RTCDRV_AllocateTimer(&xTimerForWakeUp);

#ifdef DEBUG
	Serial.setRouteLoc(1);
	Serial.begin(115200);
#endif

	/* Start watchdog */
	WDOG_Init(&wInit);

	tx_cause = RESET_TX;
	need_push = 0x5a;
}

void qsetup()
{
	sx126x.wakeup();
	sx126x.init();
	sx126x.setup_v0(TXRX_CH, MAX_DBM);
}

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

void push_data(bool cad_on)
{
	long start;
	long end;

	int e;

	uint8_t *pkt = message;

	memset(pkt, 0, PAYLOAD_LEN+6);
	pkt[0] = 0x47; pkt[1] = 0x4F; pkt[2] = 0x33;

	WDOG_Feed();

	////////////////////////////////
	pkt[15] = tx_cause;
	////////////////////////////////

	power_on_dev();		// turn on device power

	if (KEY_TX == tx_cause || RESET_TX == tx_cause ||
		TIMER_TX == tx_cause) {

		cur_smk = get_smk();
	}

	uint64_t devid = get_devid();

	uint8_t *p = (uint8_t *) &devid;

	// set devid
	int i = 0;
	for(i = 0; i < 8; i++) {
		pkt[3+i] = p[7-i];
	}

	int16_t ui16 = (int16_t)(cur_smk);
	p = (uint8_t *) &ui16;

	pkt[11] = p[1]; pkt[12] = p[0];

	cur_vbat = fetch_vbat();
	ui16 = cur_vbat * 1000;
	pkt[13] = p[1]; pkt[14] = p[0];

	float chip_temp = fetch_mcu_temp();

	// Humidity Sensor data	or Water Leak Sensor data
	pkt[20] = 0;

	// Internal Temperature of the chip
	pkt[21] = (int8_t)roundf(chip_temp);

	// Internal humidity to detect water leak of the shell
	pkt[22] = 255;

	// Internal current consumption
	pkt[23] = 0;

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

	qsetup();

#ifdef DEBUG
	start = millis();
#endif

	sx126x.enable_cad();
	e = sx126x.send(message, PAYLOAD_LEN+6, SX126x_TXMODE_SYNC);
	sx126x.set_sleep();

	if (!e) {
		// send message succesful, update the old_temp
		old_smk = cur_smk;
		tx_ok_cnt++;
	}


#ifdef DEBUG
	end = millis();

	INFO("LoRa Sent in ");
	INFOLN(end - start);

	INFO("LoRa Sent w/CAD in ");
	INFOLN(end - sx1272._startDoCad);

	INFO("Packet sent, state ");
	INFOLN(e);
#endif

	power_off_dev();
}

void loop()
{
	if (0x5a <= need_push && vbat_low == false) {
		push_data(true);
		need_push -= 1;
	}

	power_off_dev();

	/*
	 * Enable rtc timer before enter deep sleep
	 * Stop rtc timer after enter check_sensor_data()
	 */
	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypeOneshot, check_period * 1000, check_sensor, NULL);

	EMU_EnterEM2(true);
}
