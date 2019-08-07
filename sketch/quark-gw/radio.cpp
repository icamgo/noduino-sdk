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

#include "radio.h"

// be careful, max command length is 60 characters
#define MAX_CMD_LEN			100

bool optHEX = false;

void radio_setup()
{
#if 0
	sx1272.ON();		// power on the module

	// BW=125KHz, SF=12, CR=4/5, sync=0x34
	sx1272.setMode(LORA_MODE);

	// Select frequency channel
	sx1272.setChannel(DEFAULT_CH);

#ifdef PABOOST
	// Select amplifier line; PABOOST or RFO
	sx1272._needPABOOST = true;
#endif

	sx1272.setPowerDBM((uint8_t) MAX_DBM);
#else
	sx1272.sx1278_qsetup(CH_00_433, 20);
#endif

	// Set the node address and print the result
	//sx1272.setNodeAddress(LORA_ADDR);
	sx1272._nodeAddress = LORA_ADDR;

#ifdef GW_RELAY
	sx1272._enableCarrierSense = true;
	SIFS_cad_number = 6;
#endif
}

#ifdef GW_RELAY
bool RSSIonSend = true;

unsigned long startDoCad, endDoCad;
bool extendedIFS = true;
uint8_t SIFS_cad_number;
// set to 0 to disable carrier sense based on CAD
uint8_t send_cad_number = 3;
uint8_t SIFS_value[11] = { 0, 183, 94, 44, 47, 23, 24, 12, 12, 7, 4 };
uint8_t CAD_value[11] = { 0, 62, 31, 16, 16, 8, 9, 5, 3, 1, 1 };

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
#endif

int radio_available(char *cmd)
{
	int i = 0, e = 1;

	INFO_S("%s", "^$Low-level gw status ON\n");

	e = sx1272.receivePacketTimeout(MAX_TIMEOUT);

	if (e != 0 && e != 3) {
		// e = 1 or e = 2 or e > 3
		INFO_S("%s", "^$Receive error ");
		INFOLN("%d", e);

		if (e == 2) {
			// Power OFF the module
			sx1272.OFF();
			INFO_S("%s", "^$Resetting radio module\n");

			radio_setup();

			// to start over
			e = 1;
		}
	}

	if (!e && sx1272._requestACK_indicator) {
		INFO_S("%s", "^$ACK requested by ");
		INFOLN("%d", sx1272.packet_received.src);
	}

	if (!e) {

		int a = 0, b = 0;
		uint8_t p_len ;

		p_len = sx1272.getPayloadLength();

#ifdef GW_RELAY
		// here we resend the received data to the next gateway
		// set correct header information
		sx1272._nodeAddress = sx1272.packet_received.src;
		sx1272._packetNumber = sx1272.packet_received.packnum;
		sx1272.setPacketType(sx1272.packet_received.type);

		CarrierSense();

		e = sx1272.sendPacketTimeout(1,
						 sx1272.packet_received.
						 data, p_len, 10000);

		INFO_S("%s", "Packet re-sent, state ");
		INFOLN("%d", e);

		// set back the gateway address
		sx1272._nodeAddress = LORA_ADDR;
		return 0;
#else
		//sx1272.getSNR();
		sx1272.getRSSIpacket();

		// src_id,SNR,RSSI
		sprintf(cmd, "%d,%d,%d,",
			sx1272.packet_received.src,
			sx1272._SNR,
			sx1272._RSSIpacket);

		b = strlen(cmd);

		for (; a < p_len; a++, b++) {

			if (b < MAX_CMD_LEN)
				cmd[b] = (char)sx1272.packet_received.data[a];
		}

		cmd[b] = '\0';

		INFOLN("%s", cmd);

		return 1;
#endif
	} else {
		return 0;
	}
}
