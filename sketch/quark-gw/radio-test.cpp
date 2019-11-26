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

//#define GW_RELAY

#ifdef CONFIG_V0

//#define RECEIVE_ALL
//#define	RX_TIME				500
#define GW_AUTO_ACK
#define	RX_TIME				MAX_TIMEOUT
bool optHEX = true;
uint8_t loraMode = 12;

#else

#define GW_AUTO_ACK
#define	RX_TIME				MAX_TIMEOUT
bool optHEX = false;
uint8_t loraMode = 11;	// BW=125KHz, CR=4/5, SF=12

#endif

uint8_t loraAddr = 1;	// GW address

// be careful, max command length is 60 characters
#define MAX_CMD_LENGTH 100
char cmd[MAX_CMD_LENGTH] = "***";

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

#define INFO_S(fmt,param)			Serial.print(F(param))
#define INFO_HEX(fmt,param)			Serial.print(param,HEX)
#define INFO(fmt,param)				Serial.print(param)
#define INFOLN(fmt,param)			Serial.println(param)
#define FLUSHOUTPUT					Serial.flush();

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

#ifdef CONFIG_V0
char goid_buf[21];
char go_vbat[6];
char go_temp[10];

char *uint64_to_str(uint64_t n)
{
	char *dest;
	dest = goid_buf;

	dest += 20;
	*dest-- = 0;
	while (n) {
		*dest-- = (n % 10) + '0';
		n /= 10;
	}
	return dest + 1;
}

char *decode_goid(uint8_t *pkt)
{
	int a = 0, b = 0;

	uint64_t goid = 0UL;

	for (a = 3; a < 11; a++, b++) {

		*(((uint8_t *)&goid) + 7 - b) = pkt[a];
	}

	return uint64_to_str(goid);
}

/* only support .0001 */
char *ftoa(char *a, float f, int preci)
{
	long p[] =
	    {0, 10, 100, 1000, 10000};

	char *ret = a;

	long ipart = (long)f;

	//INFOLN("%d", ipart);

	itoa(ipart, a, 10);		//int16, -32,768 ~ 32,767

	while (*a != '\0')
		a++;

	*a++ = '.';

	long fpart = abs(f * p[preci] - ipart * p[preci]);

	//INFOLN("%d", fpart);

	if (fpart > 0) {
		if (fpart < p[preci]/10) {
			*a++ = '0';
		}
		if (fpart < p[preci]/100) {
			*a++ = '0';
		}
		if (fpart < p[preci]/1000) {
			*a++ = '0';
		}
	}

	itoa(fpart, a, 10);
	return ret;
}

char *decode_vbat(uint8_t *pkt)
{
	uint16_t vbat = 0;

	switch(pkt[2]) {
		case 0x31:
		case 0x32:
		case 0x33:
			vbat = pkt[13] << 8 | pkt[14];
			break;
	}

	ftoa(go_vbat, (float)(vbat / 1000.0), 3);
}

char *decode_temp(uint8_t *pkt)
{
	int16_t temp = 0;

	temp = ((pkt[11] << 8) & 0x7F) | pkt[12];

	if (pkt[11] & 0x80)
		temp = temp * -1;

	ftoa(go_temp, (float)(temp / 10.0), 1);
}

uint8_t decode_cmd(uint8_t *pkt)
{
	uint8_t cmd = 0;

	switch(pkt[2]) {

		case 0x33:
			cmd = pkt[15];
			break;
		default:
			cmd = 255;
			break;
	}

	return cmd;
}

uint8_t decode_ver(uint8_t *pkt)
{
	return pkt[2];
}
#endif

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
		uint8_t p_len;

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
		sx1272._nodeAddress = loraAddr;
#else
		sx1272.getRSSIpacket();

		sprintf(cmd, "%d,%d,%d", p_len, sx1272._RSSIpacket, sx1272._SNR);
		INFOLN("%s", cmd);

#ifdef CONFIG_V0
		for (; a < p_len; a++, b++) {

			if (optHEX) {
				if ((uint8_t) sx1272.packet_received.data[a] < 16)
					INFO_S("%s", "0");

				INFO_HEX("%X", (uint8_t) sx1272. packet_received.data[a]);
				INFO_S("%s", " ");
			} else
				INFO("%c", (char)sx1272.packet_received.data[a]);
		}

		INFOLN("%d", "$");

		sprintf(cmd, "/devid/%s/U/%s/T/%s/cmd/%d/ver/%d/rssi/%d/snr/%d",
			decode_goid(sx1272.packet_received.data),
			decode_vbat(sx1272.packet_received.data),
			decode_temp(sx1272.packet_received.data),
			decode_cmd(sx1272.packet_received.data),
			decode_ver(sx1272.packet_received.data),
			sx1272._RSSIpacket,
			sx1272._SNR);
#else
		for (; a < p_len; a++, b++) {

			cmd[b] = (char)sx1272.packet_received.data[a];
		}

		cmd[b] = '\0';

		b = strlen(cmd);

		// src_id,SNR,RSSI
		sprintf(cmd+b, "/devid/%d/snr/%d/rssi/%d",
			sx1272.packet_received.src,
			sx1272._SNR,
			sx1272._RSSIpacket);

#endif

		INFOLN("%s", cmd);
#endif
	}
}
