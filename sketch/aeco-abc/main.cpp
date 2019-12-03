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
#include "vbat.h"
#include "softi2c.h"
#include "pc10.h"
//#include "U8g2lib.h"

#include "rtcdriver.h"
#include "math.h"

/* Timer used for bringing the system back to EM0. */
RTCDRV_TimerID_t xTimerForWakeUp;

static uint32_t sample_period = 15;		/* 20s */

#define	TX_TESTING				1

// 1, 2, 3 <----> A, B, C
#define DEV_ID					2

static uint8_t need_push = 0;

#define ENABLE_CAD				1

#define node_addr				110
#define DEST_ADDR				1
#define	TX_TIME					3600		// 1000ms

#ifdef CONFIG_V0
#define TXRX_CH				CH_01_472
#define LORA_MODE			12
#else
#define TXRX_CH				CH_00_470
#define LORA_MODE			11
#endif

#define MAX_DBM					20

//#define ENABLE_SSD1306		1

//#define WITH_ACK

#define	DEBUG					1

#ifdef CONFIG_V0
uint8_t message[32] = { 0x48, 0x4F, 0x33 };
uint8_t tx_cause = 0;
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

#ifdef ENABLE_SSD1306
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);
#endif

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
	digitalWrite(10, HIGH);
}

void power_off_dev()
{
	digitalWrite(10, LOW);
}

#ifdef ENABLE_SSD1306
void draw_press(int32_t p)
{
	u8g2.setPowerSave(0);

	//u8g2.clearBuffer();		// clear the internal memory

	u8g2.setFont(u8g2_font_logisoso24_tf);	// choose a suitable font

	u8g2.firstPage();

	do {
		u8g2.drawStr(98, 48, "Pa");		// write something to the internal memory
		u8g2.setCursor(16, 48);
		u8g2.print(p);
	} while (u8g2.nextPage());

	delay(2000);

	u8g2.setPowerSave(1);

	//u8g2.sendBuffer();		// transfer internal memory to the display
}
#endif

void check_sensor(RTCDRV_TimerID_t id, void *user)
{
	(void)id;
	(void)user;

	static float old_pres = 0.0;

	RTCDRV_StopTimer(xTimerForWakeUp);

	//Serial.println("Checking...");

	//power_on_dev();		// turn on device power

	//pressure_init();	// initialization of the sensor
	//float pres = get_pressure();
	float pres = 0.0;

	//power_off_dev();

#ifdef TX_TESTING
	need_push = 0x5a;
	tx_cause = 2;
#else
	if (fabsf(pres - old_pres) > 3.0) {

		old_pres = pres;

		need_push = 0x5a;
#ifdef CONFIG_V0
		tx_cause = 1;
#endif
	}
#endif

	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypeOneshot, sample_period * 1000, check_sensor, NULL);
}

void trig_check_sensor()
{
	noInterrupts();

	need_push = 0x5a;
#ifdef CONFIG_V0
	tx_cause = 3;
#endif

	interrupts();
}

void setup()
{
	Ecode_t e;

#if 0
	/* Initialize EM23 with default parameters */
	EMU_EM23Init_TypeDef em23Init = EMU_EM23INIT_DEFAULT;
	EMU_EM23Init(&em23Init);
#endif

	// dev power ctrl
	pinMode(10, OUTPUT);

	power_off_dev();

	pinMode(0, INPUT);
	attachInterrupt(0, trig_check_sensor, FALLING);

#ifdef ENABLE_SSD1306
	float pres = get_pressure();

	u8g2.begin();

	draw_press((int32_t) pres);
#endif

	/* Initialize RTC timer. */
	RTCDRV_Init();
	RTCDRV_AllocateTimer(&xTimerForWakeUp);

	Serial.setRouteLoc(1);
	Serial.begin(115200);

	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypeOneshot, sample_period * 1000, check_sensor, NULL);
}

void qsetup()
{
	vbat_adc_init();

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

#ifdef ENABLE_SSD1306
	u8g2.begin();
#endif
}

#ifdef CONFIG_V0

uint64_t get_devid()
{
	return (11907000000ULL + DEV_ID);	// T2p
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

	float vbat = 0.0, press = 0.0;

	uint8_t r_size;

	int e;

	qsetup();

	//pressure_init();
	//press = get_pressure();		// hPa (mbar)
	press = 0.0;		// hPa (mbar)

	vbat = get_vbat();

#ifdef CONFIG_V0
	uint8_t *pkt = message;
	uint64_t devid = get_devid();

	uint8_t *p = (uint8_t *) &devid;

	pkt[0] = 0x55;
	pkt[1] = DEV_ID;

	// set devid
	int i = 0;
	for(i = 0; i < 8; i++) {
		pkt[3+i] = p[7-i];
	}

	// press/1000.0 = bar (0.1MPa), then x 100 for packet
	press /= 10.0;
	press = roundf(press);
	uint16_t ui16 = (uint16_t)press;
	p = (uint8_t *) &ui16;

	pkt[11] = p[1]; pkt[12] = p[0];

	ui16 = vbat * 1000;
	pkt[13] = p[1]; pkt[14] = p[0];

	pkt[15] = tx_cause;

	p = (uint8_t *) &tx_count;
	tx_count++;
	pkt[16] = p[1]; pkt[17] = p[0];

	ui16 = get_crc(pkt, 18);
	p = (uint8_t *) &ui16;
	pkt[18] = p[1]; pkt[19] = p[0];

#else
	char vbat_s[10], pres_s[10];
	ftoa(vbat_s, vbat, 2);
	ftoa(pres_s, press, 2);

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
	e = sx1272.sendPacketTimeout(DEST_ADDR, message, 20, TX_TIME);
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

	wire_end();
}

void loop()
{
	//INFO("Clock Freq = ");
	//INFOLN(CMU_ClockFreqGet(cmuClock_CORE));

	//INFOLN("Feed the watchdog");

	if (0x5a == need_push) {
		push_data();

		need_push = 0;
	}

	delay(50);

	power_off_dev();
	digitalWrite(SX1272_RST, LOW);

	spi_end();
	wire_end();

	EMU_EnterEM2(true);
}
