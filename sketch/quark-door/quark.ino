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
#include "vbat.h"

//#define USE_SI2301		1
#define ENABLE_CAD			1

#define	WATER_LEAK		1

#ifdef USE_SI2301
#define node_addr		251
#else
#define node_addr		100
#endif

#define USE_SX1278		1

#define DEST_ADDR		1

#define LOW_POWER

#define MAX_DBM			20
#define TXRX_CH			CH_00_433

///////////////////////////////////////////////////////////////////
//#define WITH_EEPROM
//#define WITH_APPKEY
//#define WITH_ACK
///////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////
// CHANGE HERE THE LORA MODE
#define LORAMODE		11	// BW=125KHz, SF=12, CR=4/5, sync=0x34
//////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE TIME IN SECONDS BETWEEN 2 READING & TRANSMISSION
unsigned int idlePeriod = 3600;	// 3600 seconds
///////////////////////////////////////////////////////////////////

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
#define INFO(fmt,param)				Serial.print(param)
#define INFOLN(fmt,param)			Serial.println(param)
#define FLUSHOUTPUT					Serial.flush();

#ifdef WITH_EEPROM
#include <EEPROM.h>
#endif

#ifdef WITH_ACK
#define	NB_RETRIES			2
#endif

#ifdef LOW_POWER
#define	LOW_POWER_PERIOD	8
// you need the LowPower library from RocketScream
// https://github.com/rocketscream/Low-Power
#include "LowPower.h"
unsigned int nCycle = idlePeriod / LOW_POWER_PERIOD;
#endif

uint32_t next_tx = 0L;

#ifdef WITH_EEPROM
struct sx1272config {

	uint8_t flag1;
	uint8_t flag2;
	uint8_t seq;
	// can add other fields such as LoRa mode,...
};

sx1272config my_sx1272config;
#endif

void push_alarm();
#ifdef WATER_LEAK
void push_wl_alarm();
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

void setup()
{
	int e;

#ifndef USE_SI2301
	pinMode(6, OUTPUT);
#else
	pinMode(7, OUTPUT);
#endif

	// The interrupt of door open is through D2
	pinMode(2, INPUT);

	// attach interrupt in D2, falling is door open 
	attachInterrupt(0, push_alarm, CHANGE);

#ifdef WATER_LEAK
	// The interrupt of water leak is through D3
	pinMode(3, INPUT);

	// attach interrupt in D3, falling is water leak
	attachInterrupt(1, push_wl_alarm, FALLING);
#endif

	Serial.begin(115200);

	INFO_S("%s", "Noduino Quark LoRa Node\n");

// See http://www.nongnu.org/avr-libc/user-manual/using_tools.html
// for the list of define from the AVR compiler
#ifdef __AVR_ATmega328P__
	INFO_S("%s", "ATmega328P detected\n");
#endif

	//enable the power of LoRa
	power_on_dev();

#ifdef USE_SX1278
	// Set the TX power to 11dBm
	sx1272.sx1278_qsetup(TXRX_CH, MAX_DBM);

	sx1272.setNodeAddress(node_addr);

#ifdef ENABLE_CAD
	sx1272._enableCarrierSense = true;
#endif

	INFO_S("%s", "SX1272 successfully configured\n");
#endif
}

void qsetup()
{
#ifdef USE_SX1278
	sx1272.sx1278_qsetup(TXRX_CH, MAX_DBM);
	sx1272.setNodeAddress(node_addr);

#ifdef ENABLE_CAD
	sx1272._enableCarrierSense = true;
#endif
#endif
}

void push_data(int wl_flag = 0)
{
	long startSend;
	long endSend;
	int e;
	uint8_t app_key_offset = 0;
	float vbat;
	int door = 0;
#ifdef WATER_LEAK
	int wl = 0;
#endif

	vbat = get_vbat();
	door = digitalRead(2);

#ifdef WATER_LEAK
	wl = digitalRead(3);
#endif

#ifdef WITH_APPKEY
	app_key_offset = sizeof(my_appKey);
	// set the app key in the payload
	memcpy(message, my_appKey, app_key_offset);
#endif

	uint8_t r_size;

	// the recommended format if now \!TC/22.5
	char vbat_s[10];
	ftoa(vbat_s, vbat, 2);

	// this is for testing, uncomment if you just want to test, without a real pressure sensor plugged
	//strcpy(vbat_s, "noduino");
#ifdef WATER_LEAK
	if (wl_flag == 1)
		r_size = sprintf((char *)message + app_key_offset, "\\!U/%s/WL/%d", vbat_s, wl);
	else
		r_size = sprintf((char *)message + app_key_offset, "\\!U/%s/DR/%d", vbat_s, door);
#else
	r_size = sprintf((char *)message + app_key_offset, "\\!U/%s/DR/%d", vbat_s, door);
#endif

#ifdef DEBUG
	INFO_S("%s", "Sending ");
	INFOLN("%s", (char *)(message + app_key_offset));

	INFO_S("%s", "Real payload size is ");
	INFOLN("%d", r_size);
#endif

	int pl = r_size + app_key_offset;

#ifdef ENABLE_CAD
	sx1272.CarrierSense();
#endif

	startSend = millis();

#ifdef USE_SX1278
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
			INFO_S("%s", "No ACK");

		n_retry--;

		if (n_retry)
			INFO_S("%s", "Retry");
		else
			INFO_S("%s", "Abort");

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

#ifdef DEBUG
	INFO_S("%s", "LoRa pkt size ");
	INFOLN("%d", pl);

	INFO_S("%s", "LoRa pkt seq ");
	INFOLN("%d", sx1272.packet_sent.packnum);

	INFO_S("%s", "LoRa Sent in ");
	INFOLN("%ld", endSend - startSend);

	INFO_S("%s", "LoRa Sent w/CAD in ");
	INFOLN("%ld", endSend - sx1272._startDoCad);

	INFO_S("%s", "Packet sent, state ");
	INFOLN("%d", e);

	INFO_S("%s", "Remaining ToA is ");
	INFOLN("%d", sx1272.getRemainingToA());
#endif

#endif
}

void enter_low_power()
{
	int e;

#ifdef USE_SX1278
	e = sx1272.setSleepMode();
	if (!e)
		INFO_S("%s", "Successfully switch LoRa into sleep mode\n");
	else
		INFO_S("%s", "Could not switch LoRa into sleep mode\n");
#endif

	digitalWrite(SX1272_RST, LOW);

	SPI.end();
	digitalWrite(10, LOW);
	digitalWrite(11, LOW);
	digitalWrite(12, LOW);
	digitalWrite(13, LOW);

	FLUSHOUTPUT
	delay(50);

	power_off_dev();

	for (int i = 0; i < nCycle; i++) {

		// ATmega328P, ATmega168, ATmega32U4
		LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

		//INFO_S("%s", ".");
		FLUSHOUTPUT delay(10);
	}
}

void push_alarm()
{
	pinMode(2, INPUT);

	//enable the power of LoRa
	power_on_dev();

	qsetup();

	push_data();

	enter_low_power();
}

#ifdef WATER_LEAK
void push_wl_alarm()
{
	pinMode(3, INPUT);

	//enable the power of LoRa
	power_on_dev();

	qsetup();

	push_data(1);

	enter_low_power();
}
#endif

void loop(void)
{
#ifndef LOW_POWER
	if (millis() > next_tx) {
#endif
		push_data();

#ifdef LOW_POWER

		INFO_S("%s", "Switch to power saving mode\n");
		enter_low_power();

		delay(50);

		setup();
#else
		INFOLN("%ld", next_tx);
		INFO_S("%s", "Will send next value at\n");

		next_tx = millis() + (uint32_t)idlePeriod * 1000;

		INFOLN("%ld", next_tx);
	}
#endif
}
