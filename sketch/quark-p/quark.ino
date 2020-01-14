/*
 *  Copyright (c) 2019 - 2029 MaiKe Labs
 *
 *  Based on the Arduino_LoRa_Demo_Sensor sketch
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

#include <SPI.h>
#include "sx1272.h"
#include "pressure.h"
#include "vbat.h"
#include "U8g2lib.h"

//#define	DEBUG				1

//#define USE_SI2301			1

#define ENABLE_CAD			1

#define node_addr			248

#define DEST_ADDR			1

#define USE_SX1278			1

//#define ENABLE_SSD1306		1

#define	RESET_TX			0
#define	DELTA_TX			1
#define	TIMER_TX			2

//uint8_t tx_cause = RESET_TX;

#define MAX_DBM			11
#define TXRX_CH			CH_00_433
#define LORAMODE		11	// BW=125KHz, SF=12, CR=4/5, sync=0x34

///////////////////////////////////////////////////////////////////
#define WITH_EEPROM
//#define WITH_ACK
///////////////////////////////////////////////////////////////////

//uint32_t sample_count = 0;
#define		HEARTBEAT_TIME			3600

//float old_pres = 0.0;
float cur_pres = 0.0;

uint8_t need_push = 0;

uint8_t idlePeriod = 40;	// 40 seconds

uint8_t message[50];

#ifdef DEBUG

#define INFO_S(param)				Serial.print(F(param))
#define INFO_HEX(param)				Serial.println(param,HEX)
#define INFO(param)					Serial.print(param)
#define INFOLN(param)				Serial.println(param)
#define FLUSHOUTPUT					Serial.flush();

#else

#define INFO_S(param)
#define INFO_HEX(param)
#define INFO(param)
#define INFOLN(param)
#define FLUSHOUTPUT

#endif

#ifdef WITH_EEPROM
#include <EEPROM.h>
#endif

#ifdef WITH_ACK
#define	NB_RETRIES			2
#endif

#define	LOW_POWER_PERIOD	8
// you need the LowPower library from RocketScream
// https://github.com/rocketscream/Low-Power
#include "LowPower.h"
uint32_t nCycle = idlePeriod / LOW_POWER_PERIOD;

uint32_t next_tx = 0L;

#ifdef WITH_EEPROM
struct global_param {

	uint32_t sample_count;
	float old_pres;
	uint8_t tx_cause;
	// can add other fields such as LoRa mode,...
};

struct global_param g_param;
#endif

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
#ifndef USE_SI2301
	digitalWrite(6, HIGH);
#else
	digitalWrite(7, LOW);
#endif
}

void power_off_dev()
{
#ifndef USE_SI2301
	digitalWrite(6, LOW);
#else
	digitalWrite(7, HIGH);
#endif
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

void check_sensor()
{
	// feed the watchdog

	//INFOLN("Checking the sensor...");

	g_param.sample_count++;

	if (g_param.sample_count >= HEARTBEAT_TIME/idlePeriod) {
		need_push = 0x5a;
		g_param.tx_cause = TIMER_TX;
		g_param.sample_count = 0;

		//INFOLN("It's time to push data...");
	}

	power_on_dev();		// turn on device power
	delay(2);

	pressure_init();	// initialization of the sensor
	cur_pres = get_pressure();

	power_off_dev();

	// 50.0 hPa
	if (fabsf(cur_pres - g_param.old_pres) > 50.0) {

		need_push = 0x5a;
		g_param.tx_cause = DELTA_TX;

		//INFOLN("Data is changed...");
	}

#ifdef WITH_EEPROM
	EEPROM.put(0, g_param);
#endif
}

void setup()
{
#ifndef USE_SI2301
	pinMode(6, OUTPUT);
#else
	pinMode(7, OUTPUT);
#endif
	power_off_dev();		// turn off device power

#ifdef DEBUG
	Serial.begin(115200);
#endif

	INFOLN("Seting up the device...");

#ifdef ENABLE_SSD1306
	power_on_dev();		// turn on device power
	delay(2);
	pressure_init();	// initialization of the sensor

	float pres = get_pressure();

	u8g2.begin();

	draw_press((int32_t) pres);
#endif

#ifdef WITH_EEPROM
	EEPROM.get(0, g_param);
#endif

	if (g_param.tx_cause == 0xff) {
		g_param.old_pres = 0.0;
		g_param.tx_cause = 0;
		g_param.sample_count = 0;
	}

	INFO_HEX(g_param.sample_count);

	/* bootup tx */
	if (RESET_TX == g_param.tx_cause) {
		need_push = 0x5a;
	}
}

void qsetup()
{
	power_on_dev();

#ifdef USE_SX1278
	sx1272.sx1278_qsetup(TXRX_CH, MAX_DBM);
	sx1272.setNodeAddress(node_addr);

#ifdef ENABLE_CAD
	sx1272._enableCarrierSense = true;
#endif
#endif

#ifdef ENABLE_SSD1306
	u8g2.begin();
#endif
}

void push_data()
{
	long startSend;
	long endSend;

	int e;
	float vbat = 0;

	uint8_t r_size = 0;

	vbat = get_vbat();

	if (RESET_TX == g_param.tx_cause) {
		power_on_dev();		// turn on device power

		delay(2);

		pressure_init();	// initialization of the sensor

		cur_pres = get_pressure();

		g_param.tx_cause = TIMER_TX;
	}

	char vbat_s[10], pres_s[10];
	ftoa(vbat_s, vbat, 2);
	ftoa(pres_s, cur_pres, 2);

	r_size = sprintf((char *)message, "\\!U/%s/P/%s", vbat_s, pres_s);

	INFO_S("Sending ");
	INFOLN((char *)message);

	INFO_S("Real payload size is ");
	INFOLN(r_size);

	INFO_S("tx_cause = ");
	INFOLN(g_param.tx_cause);

	qsetup();

#ifdef ENABLE_CAD
	sx1272.CarrierSense();
#endif

#ifdef DEBUG
	startSend = millis();
#endif

#ifdef USE_SX1278
	sx1272.setPacketType(PKT_TYPE_DATA);

	// Send message to the gateway and print the result
#ifdef WITH_ACK
	int n_retry = NB_RETRIES;

	do {
		e = sx1272.sendPacketTimeoutACK(DEST_ADDR,
						message, r_size);

		if (e == 3)
			INFO_S("No ACK");

		n_retry--;

		if (n_retry)
			INFO_S("Retry");
		else
			INFO_S("Abort");

	} while (e && n_retry);
#else
	e = sx1272.sendPacketTimeout(DEST_ADDR, message, r_size);
#endif

	if (!e) {
		// send message succesful, update the old_pres
		g_param.old_pres = cur_pres;

#ifdef WITH_EEPROM
		EEPROM.put(0, g_param);
#endif

		INFOLN("Update old press...");
	}

#ifdef DEBUG
	endSend = millis();

	INFO_S("LoRa Sent in ");
	INFOLN(endSend - startSend);

	INFO_S("LoRa Sent w/CAD in ");
	INFOLN(endSend - sx1272._startDoCad);

	INFO_S("Packet sent, state ");
	INFOLN(e);
#endif

#endif	// USE_SX1278

#ifdef USE_SX1278
	e = sx1272.setSleepMode();
	if (!e)
		INFO_S("Successfully switch LoRa into sleep mode\n");
	else
		INFO_S("Could not switch LoRa into sleep mode\n");
#endif

	digitalWrite(SX1272_RST, LOW);

	SPI.end();
	digitalWrite(10, LOW);
	digitalWrite(11, LOW);
	digitalWrite(12, LOW);
	digitalWrite(13, LOW);

	FLUSHOUTPUT
	delay(5);

	Wire.end();
	digitalWrite(A4, LOW);	// SDA
	digitalWrite(A5, LOW);	// SCL

	power_off_dev();
}

void loop(void)
{
	check_sensor();

	if (need_push == 0x5a) {
		push_data();

		need_push = 0;
	}

	for (uint32_t i = 0; i < nCycle; i++) {

		// ATmega328P, ATmega168, ATmega32U4
		LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

		//INFO_S("%s", ".");
		FLUSHOUTPUT delay(10);
	}

	delay(100);

	digitalWrite(SX1272_RST, LOW);
	Wire.end();
}
