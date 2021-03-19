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
#include "sht2x.h"
#include "sht3x.h"
#include "rtcdriver.h"
#include "math.h"
#include "em_wdog.h"

//#define	DEBUG					1

//#define	TX_TESTING				1

#ifdef CONFIG_V0
#include "softspi.h"
#include "sx1272.h"
#elif USE_SX126X
#include "sx126x.h"
SX126x sx126x(2,			// Pin: SPI CS,PIN06-PB08-D2
				9,			// Pin: RESET, PIN18-PC15-D9
				5,			// PIN: Busy,  PIN11-PB14-D5, The RX pin
				3			// Pin: DIO1,  PIN08-PB11-D3
);
#endif

#define CONFIG_PROTO_V33			1
//#define CONFIG_PROTO_V34			1

#ifdef CONFIG_PROTO_V33
#define ENABLE_CRYPTO				1
#endif

#ifdef ENABLE_CRYPTO
#include "crypto.h"
#endif

#define FW_VER					"Ver 1.2"

/* Timer used for bringing the system back to EM0. */
RTCDRV_TimerID_t xTimerForWakeUp;

static uint32_t sample_period = 18;		/* 20s */

static uint32_t sample_count = 0;
#define		HEARTBEAT_TIME			6600

static float old_temp = 0.0;
static float cur_temp = 0.0;
static float old_humi = 0.0;
static float cur_humi = 0.0;

static float cur_curr = 0.0;

//#define ENABLE_SHT2X			1
#define ENABLE_SHT3X			1

#ifdef ENABLE_SHT2X
#define	DELTA_HUMI				3
#elif ENABLE_SHT3X
#define	DELTA_HUMI				2
#endif

static uint32_t need_push = 0;

#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define	KEY_PIN					0		/* PIN01_PA00_D0 */

#define SDA_PIN					12		/* PIN23_PE12 */
#define SCL_PIN					13		/* PIN24_PE13 */

#define ENABLE_CAD				1

#define	TX_TIME					1800
#define DEST_ADDR				1

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
#elif USE_SX126X
#define TXRX_CH					472500000
#define MAX_DBM					22
#endif

#ifdef CONFIG_PROTO_V33
#define	PAYLOAD_LEN					30		/* 30+2+4 = 36B */
uint8_t message[PAYLOAD_LEN+6] __attribute__((aligned(4)));

#elif CONFIG_PROTO_V34
#define PAYLOAD_LEN					36		/* 36 + 6 = 42 */
uint8_t message[PAYLOAD_LEN] = { 0x47, 0x4F, 0x34 };
#endif

uint16_t tx_count = 0;

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

#ifdef MONITOR_CURRENT
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
#endif

void check_sensor(RTCDRV_TimerID_t id, void *user)
{
	(void)id;
	(void)user;

	WDOG_Feed();

	RTCDRV_StopTimer(xTimerForWakeUp);

	sample_count++;

	if (sample_count >= HEARTBEAT_TIME/sample_period) {

		need_push = 0x5a;
		tx_cause = TIMER_TX;

		sample_count = 0;

		return;
	}

#ifdef ENABLE_SHT2X
	sht2x_init(SCL_PIN, SDA_PIN);		// initialization of the sensor
	cur_temp = sht2x_get_temp();
	cur_humi = sht2x_get_humi();
#endif
#ifdef ENABLE_SHT3X
	sht3x_init(SCL_PIN, SDA_PIN);		// initialization of the sensor
	cur_temp = sht3x_get_temp();
	cur_humi = sht3x_get_humi();
#endif

#ifdef TX_TESTING
	need_push = 0x5a;
	tx_cause = TIMER_TX;
#else
	//if (fabsf(cur_temp - old_temp) > 0.5 || fabsf(cur_humi - old_humi) > 3) {
	if (fabsf(cur_humi - old_humi) > DELTA_HUMI) {

		need_push = 0x5a;

		tx_cause = DELTA_TX;

	}
#endif
}

void trig_check_sensor()
{
	need_push = 0x5a;

	tx_cause = KEY_TX;
}

void setup()
{
	Ecode_t e;

	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = wdogPeriod_32k;	/* 32k 1kHz periods should give 32 seconds */

#ifdef ENABLE_CRYPTO
	crypto_init();
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
#ifdef ENABLE_CAD
	sx1272._enableCarrierSense = true;
#endif

#elif USE_SX126X
	sx126x.wakeup();
	sx126x.init();
	sx126x.setup_v0(TXRX_CH, MAX_DBM);
#endif
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

void push_data()
{
	long startSend;
	long endSend;

	float vbat = 0.0;

	int e;

	uint8_t *pkt = message;

	memset(pkt, 0, PAYLOAD_LEN+6);
	pkt[0] = 0x47; pkt[1] = 0x4F;

	////////////////////////////////
#ifdef MONITOR_CURRENT
	cur_curr = fetch_current();

	noInterrupts();

	if (KEY_TX == tx_cause && cur_curr > 1.9)
		pkt[15] = EL_TX;
	else
		pkt[15] = tx_cause;

	interrupts();
#else
	pkt[15] = tx_cause;
#endif
	////////////////////////////////

	vbat = adc.readVbat();

	power_on_dev();

	if (KEY_TX == tx_cause || RESET_TX == tx_cause) {

	#ifdef ENABLE_SHT2X
		sht2x_init(SCL_PIN, SDA_PIN);		// initialization of the sensor
		cur_temp = sht2x_get_temp();
		cur_humi = sht2x_get_humi();
	#endif
	#ifdef ENABLE_SHT3X
		sht3x_init(SCL_PIN, SDA_PIN);		// initialization of the sensor
		cur_temp = sht3x_get_temp();
		cur_humi = sht3x_get_humi();
	#endif
	}

	INFO("Temp: ");
	INFOLN(cur_temp);

	INFO("Humi: ");
	INFOLN(cur_humi);

	uint64_t devid = get_devid();

	uint8_t *p = (uint8_t *) &devid;

	// set devid
	int i = 0;
	for(i = 0; i < 8; i++) {
		pkt[3+i] = p[7-i];
	}

	int16_t ui16 = (int16_t)(cur_humi * 10);
	p = (uint8_t *) &ui16;

	pkt[11] = p[1]; pkt[12] = p[0];

	ui16 = vbat * 1000;
	pkt[13] = p[1]; pkt[14] = p[0];

	#ifdef CONFIG_PROTO_V33
	pkt[2] = 0x33;


	float chip_temp = fetch_mcu_temp();

	// Humidity Sensor data	or Water Leak Sensor data
	pkt[20] = (int8_t)roundf(cur_temp);

	// Internal Temperature of the chip
	pkt[21] = (int8_t)roundf(chip_temp);

	// Internal humidity to detect water leak of the shell
	pkt[22] = 255;

	// Internal current consumption
#ifdef MONITOR_CURRENT
	pkt[23] = (int8_t)roundf(cur_curr);
#else
	pkt[23] = 0;
#endif

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

	#elif CONFIG_PROTO_V34
	pkt[2] = 0x34;

	pkt[16] = 8;			// dev_type

	pkt[17] = 0;			// dev_type

	/* DTF: 0 00 001 00, realtime data, int8 */
	pkt[18] = 0x04;

	/* DUF: % */
	pkt[19] = 0x15;
	pkt[20] = (int8_t) roundf(cur_humi);

	/* DTF: 0 00 101 10, realtime data, float, 4bytes */
	pkt[21] = 0x16;

	/* DUF: 'C */
	pkt[22] = 0xB;

	p = (uint8_t *) &cur_temp;
	pkt[23] = p[1];
	pkt[24] = p[0];
	pkt[25] = p[3];
	pkt[26] = p[2];

	/* DTF: 0 00 001 00, realtime data, int8 */
	pkt[27] = 0x04;
	/* DUF: % */
	pkt[28] = 0x15;
	pkt[29] = (int8_t) roundf(cur_curr);

	/* DTF: 0 00 001 00, realtime data, int16 */
	pkt[30] = 0x05;
	/* DUF: 'C */
	pkt[31] = 0x0B;

	float chip_temp = fetch_mcu_temp();

	ui16 = (int16_t)(chip_temp * 10);
	p = (uint8_t *) &ui16;

	pkt[32] = p[1]; pkt[33] = p[0];

	/* frame number */
	//pkt[34] = p[1]; pkt[35] = p[0];
	p = (uint8_t *) &tx_count;
	pkt[PAYLOAD_LEN-2] = p[1]; pkt[PAYLOAD_LEN-1] = p[0];
	tx_count++;

	//pkt[36] = p[1]; pkt[37] = p[0];
	ui16 = get_crc(pkt, PAYLOAD_LEN);
	p = (uint8_t *) &ui16;
	pkt[PAYLOAD_LEN] = p[1]; pkt[PAYLOAD_LEN+1] = p[0];

	//pkt[38] = 0; pkt[39] = 0; pkt[40] = 0; pkt[41] = 0;
	#endif

	power_on_dev();

	qsetup();

#ifdef DEBUG
	startSend = millis();
#endif

#ifdef CONFIG_V0

#ifdef ENABLE_CAD
	sx1272.CarrierSense();
#endif

	e = sx1272.sendPacketTimeout(DEST_ADDR, message, PAYLOAD_LEN+6, TX_TIME);

	sx1272.setSleepMode();
	digitalWrite(SX1272_RST, LOW);
	spi_end();
#elif USE_SX126X
	e = sx126x.send(message, PAYLOAD_LEN+6, SX126x_TXMODE_SYNC);
	sx126x.set_sleep();
#endif

	if (!e) {
		// send message succesful, update the old data
		old_temp = cur_temp;
		old_humi = cur_humi;
	}

#ifdef DEBUG
	endSend = millis();

	INFO("LoRa Sent in ");
	INFOLN(endSend - startSend);

	INFO("Packet sent, state ");
	INFOLN(e);
#endif

	power_off_dev();
}

void loop()
{
	if (0x5a == need_push) {
		push_data();

		need_push = 0;
	}

	power_off_dev();

#if defined(CONFIG_V0)
	digitalWrite(SX1272_RST, LOW);
	spi_end();
#endif

	/*
	 * Enable rtc timer before enter deep sleep
	 * Stop rtc timer after enter check_sensor_data()
	 */
	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypeOneshot, sample_period * 1000, check_sensor, NULL);

	//wire_end();
	digitalWrite(SCL_PIN, HIGH);
	digitalWrite(SDA_PIN, HIGH);

	EMU_EnterEM2(true);
}
