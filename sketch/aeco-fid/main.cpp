/* 
 *  Copyright (c) 2019 - 2029 MaiKe Labs
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "SX1272.h"

#include <stdio.h>
#include <stdint.h>

//#define DEBUG			1

#define RECEIVE_ALL

//#define RX_TIME			MAX_TIMEOUT
#define RX_TIME			300

// use the dynamic ACK feature of our modified SX1272 lib
//#define GW_AUTO_ACK

#ifdef CONFIG_V0
uint8_t loraMode = 12;
#else

// Default LoRa mode BW=125KHz, CR=4/5, SF=12
uint8_t loraMode = 11;

// Gateway address: 1
uint8_t loraAddr = 1;
#endif

// be careful, max command length is 60 characters
#define MAX_CMD_LENGTH 100
char cmd[MAX_CMD_LENGTH];

// number of retries to unlock remote configuration feature
boolean withAck = false;

bool RSSIonSend = true;

int status_counter = 0;
unsigned long startDoCad, endDoCad;
bool extendedIFS = true;
uint8_t SIFS_cad_number;
// set to 0 to disable carrier sense based on CAD
uint8_t send_cad_number = 3;
uint8_t SIFS_value[11] = { 0, 183, 94, 44, 47, 23, 24, 12, 12, 7, 4 };
uint8_t CAD_value[11] = { 0, 62, 31, 16, 16, 8, 9, 5, 3, 1, 1 };

#ifdef DEBUG

#define INFO_S(fmt,param)			Serial.print(F(param))
#define INFO_HEX(fmt,param)			Serial.print(param,HEX)
#define INFO(fmt,param)				Serial.print(param)
#define INFOLN(fmt,param)			Serial.println(param)
#define FLUSHOUTPUT					Serial.flush();

#else

#define INFO_S(fmt,param)
#define INFO(fmt,param)
#define INFOLN(fmt,param)
#define FLUSHOUTPUT	

#endif

void radio_setup()
{

#ifdef CONFIG_V0
	sx1272.setup_v0(CH_01_472, 20);
#else
	sx1272.sx1278_qsetup(CH_00_470, 20);
	sx1272._nodeAddress = loraAddr;

	sx1272._enableCarrierSense = true;
#endif

	if (loraMode > 7)
		SIFS_cad_number = 6;
	else
		SIFS_cad_number = 3;
}

char buf[21];

char *uint64_to_str(uint64_t n)
{
	char *dest;
	dest = buf;
	dest += 20;
	*dest-- = 0;
	while (n) {
		*dest-- = (n % 10) + '0';
		n /= 10;
	}
	return dest + 1;
}

void setup()
{
	int e;

	randomSeed(analogRead(14));

	Serial.begin(115200);

	// turn on the device power
	//pinMode(6, OUTPUT);
	//digitalWrite(6, HIGH);

	pinMode(7, OUTPUT);
	digitalWrite(7, LOW);

	radio_setup();
}


long long goid;

int flag = 1;

void send_id()
{
	if (goid != 0) {
		Serial.println(uint64_to_str(goid));

		flag--;
		if (flag <= 0)
			goid = 0;
	}
}

void loop(void)
{
	int i = 0, e;
	
	e = 1;

	if (Serial.available() && Serial.read() == 'G')
		send_id();

	if (status_counter == 60 || status_counter == 0) {
		INFO_S("%s", "^$Low-level gw status ON\n");
		status_counter = 0;
	}

	// check if we received data from the receiving LoRa module
#ifdef RECEIVE_ALL
	e = sx1272.receiveAll(RX_TIME);
#else
#ifdef GW_AUTO_ACK

	e = sx1272.receivePacketTimeout(RX_TIME);

	status_counter++;

	if (e != 0 && e != 3) {
		INFO_S("%s", "^$Receive error ");
		INFOLN("%d", e);

		if (e == 2) {
			// Power OFF the module
			sx1272.OFF();
			INFO_S("%s", "^$Resetting radio module\n");

			radio_setup();

			// to start over
			status_counter = 0;
			e = 1;
		}
		FLUSHOUTPUT;
	}

	if (!e && sx1272._requestACK_indicator) {
		INFO_S("%s", "^$ACK requested by ");
		INFOLN("%d", sx1272.packet_received.src);
		FLUSHOUTPUT;
	}
#else
	// OBSOLETE normally we always use GW_AUTO_ACK
	// Receive message
	if (withAck)
		e = sx1272.receivePacketTimeoutACK(RX_TIME);
	else
		e = sx1272.receivePacketTimeout(RX_TIME);

#endif // gw_auto_ack
#endif // receive_all

	if (!e) {

		int a = 0, b = 0;
		uint8_t tmp_length;

		tmp_length = sx1272.getPayloadLength();

		uint8_t *p = sx1272.packet_received.data;


		if (p[2] == 0x33 && p[15] == 0x03 && sx1272._RSSIpacket > -60) {

#ifdef DEBUG
			sprintf(cmd, "rssi = %d, %d", sx1272._RSSIpacket, sx1272._SNR);
			Serial.println(cmd);
#endif

			for (a = 3; a < 11; a++, b++) {
#ifdef DEBUG
				if ((uint8_t) sx1272.packet_received.data[a] < 16)
					Serial.print("0");
				Serial.print((uint8_t) sx1272.packet_received.data[a], HEX);
#endif

				*(((uint8_t *)&goid) + 7 - b) = sx1272.packet_received.data[a];
			}

#ifdef DEBUG
			for (a=0; a<8; a++) {

				b = *(((uint8_t *)&goid)+a);
				if(b < 16) Serial.print("0");
				Serial.print(b, HEX);
			}

			Serial.println("");

			Serial.println(uint64_to_str(goid));
#endif
			flag = 1;
		}
	}
}
