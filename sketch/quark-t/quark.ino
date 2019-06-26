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
#include "pt1000.h"
#include "vbat.h"

//#define USE_SI2301		1
#define ENABLE_CAD			1

#ifdef USE_SI2301
#define node_addr		251
#else
#define node_addr		252
#endif

#define USE_SX1278		1
#define DEST_ADDR		1

#define LOW_POWER

///////////////////////////////////////////////////////////////////
//#define WITH_EEPROM
//#define WITH_APPKEY
//#define WITH_ACK
//#define LOW_POWER_TEST
///////////////////////////////////////////////////////////////////

// IMPORTANT SETTINGS
///////////////////////////////////////////////////////////////////
// please uncomment only 1 choice
//#define ETSI_EUROPE_REGULATION
//#define FCC_US_REGULATION
//#define SENEGAL_REGULATION
#define LONG_RANG_TESTING
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// please uncomment only 1 choice
//#define BAND868
//#define BAND900
#define BAND433
//#define BAND470
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// uncomment if the rf output of your radio module use the PABOOST
// line instead of the RFO line
#define PABOOST
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE LORA MODE
#define LORAMODE		11	// BW=125KHz, SF=12, CR=4/5, sync=0x34
//////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE TIME IN SECONDS BETWEEN 2 READING & TRANSMISSION
unsigned int idlePeriod = 90;	// 90 seconds
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

#ifdef ETSI_EUROPE_REGULATION
#define MAX_DBM 14
// previous way for setting output power
// char powerLevel='M';
#elif defined SENEGAL_REGULATION
#define MAX_DBM 10
// previous way for setting output power
// 'H' is actually 6dBm, so better to use the new way to set output power
// char powerLevel='H';
#elif defined FCC_US_REGULATION
#define MAX_DBM 14
#elif defined LONG_RANG_TESTING
#define MAX_DBM 20
#endif

#ifdef BAND868
#ifdef SENEGAL_REGULATION
const uint32_t DEFAULT_CHANNEL = CH_04_868;
#else
const uint32_t DEFAULT_CHANNEL = CH_10_868;
#endif
#elif defined BAND900
//const uint32_t DEFAULT_CHANNEL=CH_05_900;
// For HongKong, Japan, Malaysia, Singapore, Thailand, Vietnam: 920.36MHz     
const uint32_t DEFAULT_CHANNEL = CH_08_900;
#elif defined BAND433
const uint32_t DEFAULT_CHANNEL = CH_00_433;	// 433.3MHz
//const uint32_t DEFAULT_CHANNEL = CH_03_433;	// 434.3MHz
#elif defined BAND470
const uint32_t DEFAULT_CHANNEL = CH_00_470;	// 470.0MHz
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

	Serial.begin(115200);

	INFO_S("%s", "Noduino Quark LoRa Node\n");

// See http://www.nongnu.org/avr-libc/user-manual/using_tools.html
// for the list of define from the AVR compiler
#ifdef __AVR_ATmega328P__
	INFO_S("%s", "ATmega328P detected\n");
#endif

	power_on_dev();		// turn on device power

	pt1000_init();		// initialization of the sensor

#if 0
#ifdef USE_SX1278
	sx1272.ON();		// power on the module

#ifdef WITH_EEPROM
	// get config from EEPROM
	EEPROM.get(0, my_sx1272config);

	// found a valid config?
	if (my_sx1272config.flag1 == 0x12 && my_sx1272config.flag2 == 0x34) {
		INFO_S("%s", "Get back previous sx1272 config\n");

		// set sequence number for SX1272 library
		sx1272._packetNumber = my_sx1272config.seq;
		INFO_S("%s", "Using packet sequence number of ");
		INFOLN("%d", sx1272._packetNumber);
	} else {
		// otherwise, write config and start over
		my_sx1272config.flag1 = 0x12;
		my_sx1272config.flag2 = 0x34;
		my_sx1272config.seq = sx1272._packetNumber;
	}
#endif

	// We use the LoRaWAN mode:
	// BW=125KHz, SF=12, CR=4/5, sync=0x34
	e = sx1272.setMode(LORAMODE);
	INFO_S("%s", "Setting Mode: state ");
	INFOLN("%d", e);

	// Select frequency channel
	e = sx1272.setChannel(DEFAULT_CHANNEL);
	INFO_S("%s", "Setting Channel: state ");
	INFOLN("%d", e);

	// Select amplifier line; PABOOST or RFO
#ifdef PABOOST
	sx1272._needPABOOST = true;
#endif

	e = sx1272.setPowerDBM((uint8_t) MAX_DBM);
	INFO_S("%s", "Setting Power: state ");
	INFOLN("%d", e);

	// Set the node address and print the result
	e = sx1272.setNodeAddress(node_addr);
	INFO_S("%s", "Setting node addr: state ");
	INFOLN("%d", e);

#ifdef ENABLE_CAD
	// enable carrier sense
	sx1272._enableCarrierSense = true;
#endif

#ifdef LOW_POWER
	// TODO: with low power, when setting the radio module in sleep mode
	// there seem to be some issue with RSSI reading
	//sx1272._RSSIonSend = false;
#endif
#endif

#else
	sx1272.sx1278_qsetup(CH_00_433, 11);

	sx1272.setNodeAddress(node_addr);

#ifdef ENABLE_CAD
	sx1272._enableCarrierSense = true;
#endif

#endif

	INFO_S("%s", "SX1272 successfully configured\n");
}

void qsetup()
{
	pt1000_init();	// initialization of the sensor

#if 0
#ifdef USE_SX1278
	sx1272.ON();		// power on the module

	// BW=125KHz, SF=12, CR=4/5, sync=0x34
	sx1272.setMode(LORAMODE);

	// Select frequency channel
	sx1272.setChannel(DEFAULT_CHANNEL);

#ifdef PABOOST
	// Select amplifier line; PABOOST or RFO
	sx1272._needPABOOST = true;
#endif

	sx1272.setPowerDBM((uint8_t) MAX_DBM);

	// Set the node address and print the result
	sx1272.setNodeAddress(node_addr);

#ifdef ENABLE_CAD
	// enable carrier sense
	sx1272._enableCarrierSense = true;
#endif

#endif
#endif
	sx1272.sx1278_qsetup(CH_00_433, 11);
	sx1272.setNodeAddress(node_addr);

#ifdef ENABLE_CAD
	sx1272._enableCarrierSense = true;
#endif
}

void loop(void)
{
	long startSend;
	long endSend;
	uint8_t app_key_offset = 0;
	int e;
	float temp, vbat;

#ifndef LOW_POWER
	// 600000+random(15,60)*1000
	if (millis() > next_tx) {
#endif

		temp = pt1000_get_temp();
		vbat = get_vbat();

		INFO_S("%s", "Temperature is ");
		INFOLN("%f", temp);

#ifdef WITH_APPKEY
		app_key_offset = sizeof(my_appKey);
		// set the app key in the payload
		memcpy(message, my_appKey, app_key_offset);
#endif

		uint8_t r_size;

		// the recommended format if now \!TC/22.5
		char vbat_s[10], temp_s[10];
		ftoa(vbat_s, vbat, 2);
		ftoa(temp_s, temp, 2);

		// this is for testing, uncomment if you just want to test, without a real pressure sensor plugged
		//strcpy(vbat_s, "noduino");
		r_size = sprintf((char *)message + app_key_offset, "\\!U/%s/T/%s", vbat_s, temp_s);

		INFO_S("%s", "Sending ");
		INFOLN("%s", (char *)(message + app_key_offset));

		INFO_S("%s", "Real payload size is ");
		INFOLN("%d", r_size);

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

#ifdef LOW_POWER
		INFO_S("%s", "Switch to power saving mode\n");

#ifdef USE_SX1278
		e = sx1272.setSleepMode();
		if (!e)
			INFO_S("%s", "Successfully switch LoRa into sleep mode\n");
		else
			INFO_S("%s", "Could not switch LoRa into sleep mode\n");
#endif

		//sx1272.reset();
		//sx1272.OFF();

		digitalWrite(SX1272_RST, LOW);

		SPI.end();
		digitalWrite(10, LOW);
		digitalWrite(11, LOW);
		digitalWrite(12, LOW);
		digitalWrite(13, LOW);

		FLUSHOUTPUT
		delay(50);

		Wire.end();
		digitalWrite(A4, LOW);	// SDA
		digitalWrite(A5, LOW);	// SCL

		power_off_dev();

		for (int i = 0; i < nCycle; i++) {

			// ATmega328P, ATmega168, ATmega32U4
			LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

			//INFO_S("%s", ".");
			FLUSHOUTPUT delay(10);
		}

		delay(50);

		power_on_dev();
		delay(100);

		setup();
#else
		INFOLN("%ld", next_tx);
		INFO_S("%s", "Will send next value at\n");

		// use a random part also to avoid collision
		next_tx = millis() + (uint32_t)idlePeriod * 1000
				+ (uint32_t)random(10, 60) * 100;

		INFOLN("%ld", next_tx);
	}
#endif
}
