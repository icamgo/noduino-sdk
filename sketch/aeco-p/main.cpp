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

/* 20s */
static uint32_t sensor_period = 90;

#define	TX_TESTING				1

//float pres = 0.0;

volatile uint8_t need_push = 0;


//#define ENABLE_CAD			1

#define node_addr		106

#define DEST_ADDR		1

//#define ENABLE_SSD1306		1

#define MAX_DBM			20
#define TXRX_CH			CH_00_470

///////////////////////////////////////////////////////////////////
//#define WITH_EEPROM
//#define WITH_APPKEY
//#define WITH_ACK
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE LORA MODE
#define LORAMODE		11	// BW=125KHz, SF=12, CR=4/5, sync=0x34
//////////////////////////////////////////////////////////////////

#ifdef WITH_APPKEY
// CHANGE HERE THE APPKEY, BUT IF GW CHECKS FOR APPKEY, MUST BE
// IN THE APPKEY LIST MAINTAINED BY GW.
uint8_t my_appKey[4] = { 5, 6, 8, 8 };
#endif

///////////////////////////////////////////////////////////////////
// IF YOU SEND A LONG STRING, INCREASE THE SIZE OF MESSAGE
uint8_t message[50];
///////////////////////////////////////////////////////////////////


#define INFO_S(fmt,param)			Serial.print(F(param))
#define INFO(param)					Serial.print(param)
#define INFOLN(param)				Serial.println(param)

#ifdef WITH_EEPROM
#include <EEPROM.h>
#endif

#ifdef WITH_ACK
#define	NB_RETRIES			2
#endif

#ifdef WITH_EEPROM
struct sx1272config {

	uint8_t flag1;
	uint8_t flag2;
	uint8_t seq;
	// can add other fields such as LoRa mode,...
};

sx1272config my_sx1272config;
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

	Serial.println("Checking...");

	power_on_dev();		// turn on device power

	pressure_init();	// initialization of the sensor

	float pres = get_pressure();

	power_off_dev();

#ifdef TX_TESTING
	need_push = 0x5a;
#else
	if (fabsf(pres - old_pres) > 10.0) {

		old_pres = pres;

		need_push = 0x5a;

	}
#endif

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

	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypeOneshot, sensor_period * 1000, check_sensor, NULL);
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

	float vbat = 0.0, press = 0.0;

	uint8_t r_size;

	uint8_t app_key_offset = 0;
	int e;

	qsetup();

	pressure_init();
	press = get_pressure();

	vbat = get_vbat();

#ifdef WITH_APPKEY
	app_key_offset = sizeof(my_appKey);
	// set the app key in the payload
	memcpy(message, my_appKey, app_key_offset);
#endif

	char vbat_s[10], pres_s[10];
	ftoa(vbat_s, vbat, 2);
	ftoa(pres_s, press, 2);

	r_size = sprintf((char *)message + app_key_offset, "\\!U/%s/P/%s", vbat_s, pres_s);

	INFO("Sending ");
	INFOLN((char *)(message + app_key_offset));

	INFO("Real payload size is ");
	INFOLN(r_size);

	int pl = r_size + app_key_offset;

#ifdef ENABLE_CAD
	sx1272.CarrierSense();
#endif

	startSend = millis();

#ifdef WITH_APPKEY
	// indicate that we have an appkey
	sx1272.setPacketType(PKT_TYPE_DATA | PKT_FLAG_DATA_WAPPKEY);
#else
	// just a simple data packet
	sx1272.setPacketType(PKT_TYPE_DATA);
#endif

	// Send message to the gateway and print the result
	// with the app key if this feature is enabled
#ifdef WITH_ACK
	int n_retry = NB_RETRIES;

	do {
		e = sx1272.sendPacketTimeoutACK(DEST_ADDR,
						message, pl);

		if (e == 3)
			INFO("No ACK");

		n_retry--;

		if (n_retry)
			INFO("Retry");
		else
			INFO("Abort");

	} while (e && n_retry);
#else
	e = sx1272.sendPacketTimeout(DEST_ADDR, message, pl);
#endif
	endSend = millis();

#ifdef WITH_EEPROM
	// save packet number for next packet in case of reboot
	my_sx1272config.seq = sx1272._packetNumber;
	EEPROM.put(0, my_sx1272config);
#endif

	INFO("LoRa pkt size ");
	INFOLN(pl);

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
