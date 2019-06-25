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

#define PABOOST

//#define BAND470
#define BAND433

#ifndef MAX_DBM
#define MAX_DBM		20
#endif

//#define SHOW_FREEMEMORY
//#define GW_RELAY
//#define RECEIVE_ALL 

// use the dynamic ACK feature of our modified SX1272 lib
#define GW_AUTO_ACK

#ifdef BAND433
const uint32_t DEFAULT_CHANNEL = CH_00_433;	// 433.3MHz
//const uint32_t DEFAULT_CHANNEL = CH_03_433;	// 434.3MHz
#elif defined BAND470
const uint32_t DEFAULT_CHANNEL = CH_00_470;	// 470.0MHz
#endif


// Default LoRa mode BW=125KHz, CR=4/5, SF=12
uint8_t loraMode = 11;

// Gateway address: 1
#define LORA_ADDR	1
uint8_t loraAddr = LORA_ADDR;

// be careful, max command length is 60 characters
#define MAX_CMD_LENGTH 100
char cmd[MAX_CMD_LENGTH] = "***";

// number of retries to unlock remote configuration feature
boolean withAck = false;

bool radioON = true;
bool RSSIonSend = true;

int status_counter = 0;
unsigned long startDoCad, endDoCad;
bool extendedIFS = true;
uint8_t SIFS_cad_number;
// set to 0 to disable carrier sense based on CAD
uint8_t send_cad_number = 3;
uint8_t SIFS_value[11] = { 0, 183, 94, 44, 47, 23, 24, 12, 12, 7, 4 };
uint8_t CAD_value[11] = { 0, 62, 31, 16, 16, 8, 9, 5, 3, 1, 1 };

bool optHEX = false;

#define INFO_S(fmt,param)			Serial.print(F(param))
#define INFO_HEX(fmt,param)			Serial.print(param,HEX)
#define INFO(fmt,param)				Serial.print(param)
#define INFOLN(fmt,param)			Serial.println(param)
#define FLUSHOUTPUT					Serial.flush();

void radio_setup()
{
#if 0
	sx1272.ON();		// power on the module

	// BW=125KHz, SF=12, CR=4/5, sync=0x34
	sx1272.setMode(loraMode);

	// Select frequency channel
	sx1272.setChannel(DEFAULT_CHANNEL);

#ifdef PABOOST
	// Select amplifier line; PABOOST or RFO
	sx1272._needPABOOST = true;
#endif

	sx1272.setPowerDBM((uint8_t) MAX_DBM);
#else
	sx1272.sx1278_qsetup(CH_00_433);
#endif

	// Set the node address and print the result
	//sx1272.setNodeAddress(loraAddr);
	sx1272._nodeAddress = loraAddr;

	sx1272._enableCarrierSense = true;

	if (loraMode > 7)
		SIFS_cad_number = 6;
	else
		SIFS_cad_number = 3;
}

void setup()
{
	int e;

	randomSeed(analogRead(14));

	Serial.begin(115200);

	radio_setup();
}

/*
 * We could use the CarrierSense function added in the SX1272 library,
 * but it is more convenient to duplicate it here so that we could easily
 * modify it for testing in v1.5 the "only once" behavior is implemented
 * for the gateway when it transmit downlink packets to avoid blocking
 * the gateway on a busy channel. Therefore from v1.5 the implementation
 * differs from the carrier sense function added in the SX1272 library
*/
int CarrierSense(bool onlyOnce = false)
{

	int e;
	bool carrierSenseRetry = false;

	if (send_cad_number) {
		do {
			do {

				// check for free channel (SIFS/DIFS)        
				startDoCad = millis();
				e = sx1272.doCAD(send_cad_number);
				endDoCad = millis();

				INFO_S("%s", "--> CAD duration ");
				INFOLN("%ld", endDoCad - startDoCad);

				if (!e) {
					INFO_S("%s", "OK1\n");

					if (extendedIFS) {
						// wait for random number of CAD
						uint8_t w = random(1, 8);

						INFO_S("%s", "--> waiting for ");
						INFO("%d", w);
						INFO_S("%s", " CAD = ");
						INFOLN("%d", CAD_value[loraMode] * w);

						delay(CAD_value[loraMode] * w);

						// check for free channel (SIFS/DIFS) once again
						startDoCad = millis();
						e = sx1272.doCAD(send_cad_number);
						endDoCad = millis();

						INFO_S("%s", "--> CAD duration ");
						INFOLN("%ld", endDoCad - startDoCad);

						if (!e)
							INFO_S("%s", "OK2\n");
						else
							INFO_S("%s", "###2\n");
					}
				} else {
					INFO_S("%s", "###1\n");

					// if we have "only once" behavior then exit here to not have retries
					if (onlyOnce)
						return 1;

					// wait for random number of DIFS
					uint8_t w = random(1, 8);

					INFO_S("%s", "--> waiting for ");
					INFO("%d", w);
					INFO_S("%s", " DIFS (DIFS=3SIFS) = ");
					INFOLN("%d", SIFS_value[loraMode] * 3 * w);

					delay(SIFS_value[loraMode] * 3 * w);

					INFO_S("%s", "--> retry\n");
				}

			} while (e);

			// CAD is OK, but need to check RSSI
			if (RSSIonSend) {

				e = sx1272.getRSSI();

				uint8_t rssi_retry_count = 10;

				if (!e) {

					INFO_S("%s", "--> RSSI ");
					INFOLN("%d", sx1272._RSSI);

					while (sx1272._RSSI > -90
					       && rssi_retry_count) {

						delay(1);
						sx1272.getRSSI();
						INFO_S("%s", "--> RSSI ");
						INFOLN("%d", sx1272._RSSI);
						rssi_retry_count--;
					}
				} else
					INFO_S("%s", "--> RSSI error\n");

				if (!rssi_retry_count)
					carrierSenseRetry = true;
				else
					carrierSenseRetry = false;
			}

		} while (carrierSenseRetry);
	}

	return 0;
}

void loop(void)
{
	int i = 0, e;

	e = 1;

	if (status_counter == 60 || status_counter == 0) {
		INFO_S("%s", "^$Low-level gw status ON\n");
		status_counter = 0;
	}

	// check if we received data from the receiving LoRa module
#ifdef RECEIVE_ALL
	e = sx1272.receiveAll(MAX_TIMEOUT);
#else
#ifdef GW_AUTO_ACK

	e = sx1272.receivePacketTimeout(MAX_TIMEOUT);

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
		e = sx1272.receivePacketTimeoutACK(MAX_TIMEOUT);
	else
		e = sx1272.receivePacketTimeout(MAX_TIMEOUT);

#endif // gw_auto_ack
#endif // receive_all

	if (!e) {

		int a = 0, b = 0;
		uint8_t tmp_length;

		tmp_length = sx1272.getPayloadLength();

#ifdef GW_RELAY
		// here we resend the received data to the next gateway
		// set correct header information
		sx1272._nodeAddress = sx1272.packet_received.src;
		sx1272._packetNumber = sx1272.packet_received.packnum;
		sx1272.setPacketType(sx1272.packet_received.type);

		CarrierSense();

		e = sx1272.sendPacketTimeout(1,
						 sx1272.packet_received.
						 data, tmp_length, 10000);

		INFO_S("%s", "Packet re-sent, state ");
		INFOLN("%d", e);

		// set back the gateway address
		sx1272._nodeAddress = loraAddr;
#else
		//sx1272.getSNR();
		sx1272.getRSSIpacket();

		// provide a short output for external program to have information about the received packet
		// src_id,seq,len,SNR,RSSI
		sprintf(cmd, "%d,%d,%d,%d,",
			sx1272.packet_received.dst,
			sx1272.packet_received.type,
			sx1272.packet_received.src,
			sx1272.packet_received.packnum);
		INFOLN("%s", cmd);

		sprintf(cmd, "%d,%d,%d\n", tmp_length, sx1272._SNR, sx1272._RSSIpacket);
		INFOLN("%s", cmd);

		for (; a < tmp_length; a++, b++) {

			if (optHEX) {
				if ((uint8_t) sx1272.packet_received.data[a] < 16)
					INFO_S("%s", "0");

				INFO_HEX("%X", (uint8_t) sx1272. packet_received.data[a]);
				INFO_S("%s", " ");
			} else
				INFO("%c", (char)sx1272.packet_received.data[a]);

			if (b < MAX_CMD_LENGTH)
				cmd[b] = (char)sx1272.packet_received.data[a];
		}

		// strlen(cmd) will be correct as only the payload is copied
		cmd[b] = '\0';

		INFOLN("%d", " ");
#endif
	}
}
