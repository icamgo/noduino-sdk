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
#include "sht2x.h"
//#include "U8g2lib.h"
#include "rtcdriver.h"
#include "math.h"
#include "em_wdog.h"

/* Timer used for bringing the system back to EM0. */
RTCDRV_TimerID_t xTimerForWakeUp;

/* 20s */
static uint32_t sensor_period = 60;

#define	TX_TESTING				1
#define ENABLE_SHT2X			1

static uint32_t need_push = 0;

#define ENABLE_CAD				1

#define node_addr				107

#define DEST_ADDR				1
#define	TX_TIME					2100		// 1800ms

//#define ENABLE_SSD1306		1

#define MAX_DBM			20
#define TXRX_CH			CH_00_470

//#define WITH_ACK

uint8_t message[50];

#define INFO_S(fmt,param)			Serial.print(F(param))
#define INFO(param)					Serial.print(param)
#define INFOLN(param)				Serial.println(param)
#define INFOHEX(param)				Serial.println(param, HEX)

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

	static float old_temp = 0.0, old_humi = 0.0;
	float temp = 0.0, humi = 0.0;

	WDOG_Feed();

	RTCDRV_StopTimer(xTimerForWakeUp);

	//INFOLN("Checking...");
	//power_on_dev();		// turn on device power

#ifdef ENABLE_SHT2X
	wire_begin(SW_SCL, SW_SDA);
	sht2x_init();		// initialization of the sensor
	temp = sht2x_get_temp();
	humi = sht2x_get_humi();
#endif

	//power_off_dev();

#ifdef TX_TESTING
	need_push = 0x5a;
#else
	if (fabsf(temp - old_temp) > 0.3 || fabsf(humi - old_humi) > 1) {

		old_humi = humi;
		old_temp = temp

		need_push = 0x5a;
	}
#endif
	//INFOLN("Checking OK...");

	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypeOneshot, sensor_period * 1000, check_sensor, NULL);
}

void trig_check_sensor()
{
	noInterrupts();

	need_push = 0x5a;

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
	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = wdogPeriod_128k;	/* 32k 1kHz periods should give 128 seconds */

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

#ifdef DEBUG
	Serial.setRouteLoc(1);
	Serial.begin(115200);
#endif

	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypeOneshot, sensor_period * 1000, check_sensor, NULL);

	/* Start watchdog */
	WDOG_Init(&wInit);

	/* bootup tx */
	//tx_cause = 0;
	need_push = 0x5a;
}

void qsetup()
{
	vbat_adc_init();

	power_on_dev();		// turn on device power

	sx1272.sx1278_qsetup(TXRX_CH, MAX_DBM);
	sx1272.setNodeAddress(node_addr);

#ifdef ENABLE_CAD
	sx1272._enableCarrierSense = true;
#endif

#ifdef ENABLE_SSD1306
	u8g2.begin();
#endif
}

void push_data()
{
	long startSend;
	long endSend;

	float vbat = 0.0;
	float temp = 0.0, humi = 0.0;

	uint8_t r_size;

	int e;

	qsetup();

#ifdef ENABLE_SHT2X
	wire_begin(SW_SCL, SW_SDA);
	sht2x_init();		// initialization of the sensor
	temp = sht2x_get_temp();
	humi = sht2x_get_humi();
#endif

	vbat = get_vbat();

	char vbat_s[10], temp_s[10], humi_s[10];

	ftoa(vbat_s, vbat, 2);
	ftoa(temp_s, temp, 2);
	ftoa(humi_s, humi, 0);

	r_size = sprintf((char *)message, "\\!U/%s/T/%s/H/%s",
				vbat_s, temp_s, humi_s);

	INFO("Sending ");
	INFOLN((char *)(message));

	INFO("Real payload size is ");
	INFOLN(r_size);

#ifdef ENABLE_CAD
	sx1272.CarrierSense();
#endif

	startSend = millis();

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
	e = sx1272.sendPacketTimeout(DEST_ADDR, message, r_size, TX_TIME);
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
	INFO("Clock Freq = ");
	INFOLN(CMU_ClockFreqGet(cmuClock_CORE));

	INFO("need_push = ");
	INFOHEX(need_push);

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
