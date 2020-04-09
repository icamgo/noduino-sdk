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

#include "softspi.h"
#include "sx1272.h"

#include <stdio.h>
#include <stdint.h>

#define	DEBUG					1

#define	PWR_CTRL_PIN			8		/* PIN17_PC14_D8 */
#define	KEY_PIN					0		/* PIN01_PA00_D0 */

#define SYNCWORD_DEFAULT		0x12
#define SYNCWORD_LORAWAN		0x34
#define SYNCWORD_ABC			0x55

#define	ENABLE_CAD				1

#ifdef CONFIG_V0

#define RECEIVE_ALL
#define TXRX_CH					CH_01_472
#define RX_TIME					330
#define	TX_TIME					500		// 100ms
#define LORA_MODE				12

#else

#define TXRX_CH					CH_00_470		// 470.0MHz
#define RX_TIME					MAX_TIMEOUT
// Default LoRa mode BW=125KHz, CR=4/5, SF=12
#define LORA_MODE				11
#define MAX_CMD_LENGTH			100
char cmd[MAX_CMD_LENGTH];
#endif

#define DEST_ADDR				1
#define MAX_DBM					20

int8_t rx_intval = 125;
int8_t rx_window = 5;
int8_t rx_flag = 0;

char msg_buf[2][24];

/*
 * Output Mode:
 *
 *   0x0: rx all messages
 *   0x1: only rx the tagged message
 *   0x2: only rx trigged message
*/
int omode = 1;
int old_omode = 1;

int key_time = 0;

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
	sx1272.setup_v0(TXRX_CH, MAX_DBM);
	//sx1272.setPreambleLength(6);
	//sx1272.setSyncWord(SYNCWORD_ABC);
#else
	sx1272.sx1278_qsetup(TXRX_CH, MAX_DBM);
	sx1272._nodeAddress = DEST_ADDR;
#endif

#ifdef ENABLE_CAD
	sx1272._enableCarrierSense = true;
#endif
}

#ifdef CONFIG_V0
char dev_id[24];

char *uint64_to_str(uint64_t n)
{
	char *dest = dev_id;

	dest += 20;
	*dest-- = 0;
	while (n) {
		*dest-- = (n % 10) + '0';
		n /= 10;
	}

	strcpy(dev_id, dest+1);

	return dest + 1;
}

char *decode_devid(uint8_t *pkt)
{
	int a = 0, b = 0;

	uint64_t devid = 0UL;

	for (a = 3; a < 11; a++, b++) {

		*(((uint8_t *)&devid) + 7 - b) = pkt[a];
	}

	return uint64_to_str(devid);
}
#endif

void change_omode()
{
	omode++;
	omode %= 3;
	INFO("%s", "omode: ");
	INFOLN("%d", omode);
}

void power_on_dev()
{
	digitalWrite(PWR_CTRL_PIN, HIGH);
}

void power_off_dev()
{
	digitalWrite(PWR_CTRL_PIN, LOW);
}

void setup()
{
	int e;

	// Key connected to D0
	pinMode(KEY_PIN, INPUT);
	attachInterrupt(KEY_PIN, change_omode, FALLING);

	// dev power ctrl
	pinMode(PWR_CTRL_PIN, OUTPUT);

	power_on_dev();

#ifdef DEBUG
	Serial.setRouteLoc(1);
	Serial.begin(115200);
#endif

	radio_setup();
}

void loop(void)
{
	int e = 1;
	static int c = 0;

	if (digitalRead(KEY_PIN) == 0) {
		// x 200ms
		key_time++;
	} else {
		key_time = 0;
	}

	if (key_time > 8) {

		key_time = 0;

		INFOLN("%s", "Key long time pressed, enter deep sleep...");
		delay(300);

		sx1272.setSleepMode();
		digitalWrite(SX1272_RST, LOW);

		spi_end();
		digitalWrite(10, LOW);
		digitalWrite(11, LOW);
		digitalWrite(12, LOW);
		digitalWrite(13, LOW);

		// dev power off
		power_off_dev();

		EMU_EnterEM2(true);

		setup();
	}

	// check if we received data from the receiving LoRa module
#ifdef RECEIVE_ALL
	e = sx1272.receiveAll(RX_TIME);
#else
	e = sx1272.receivePacketTimeout(RX_TIME);

	if (e != 0 && e != 3) {
		// e = 1 or e = 2 or e > 3
		INFO_S("%s", "^$Receive error ");
		INFOLN("%d", e);

		if (e == 2) {
			// Power OFF the module
			sx1272.reset();
			INFO_S("%s", "^$Resetting radio module\n");

			radio_setup();

			// to start over
			e = 1;
		}
	}
#endif

	if (!e) {

		sx1272.getRSSIpacket();

		uint8_t pkt_len = sx1272.getPayloadLength();

		uint8_t *rx_pkt = sx1272.packet_received.data;

		decode_devid(rx_pkt);

		if (strcmp(dev_id, "") == 0) {
			// lora module unexpected error
			sx1272.reset();

			INFO_S("%s", "Resetting lora module\n");

			radio_setup();

	//	} else if (strcmp(dev_id, "11902041155") == 0) {
		} else if (strcmp(dev_id, "11902460803") == 0) {

			// Add a tag. It's relayed by cc
			rx_pkt[15] |= 0x80;

#ifdef ENABLE_CAD
			sx1272.CarrierSense();
#endif
			// here we resend the received data to the next gateway
			e = sx1272.sendPacketTimeout(DEST_ADDR, rx_pkt, 24, TX_TIME);

			INFO_S("%s", "Packet re-sent, state ");
			INFOLN("%d", e);

			// set back the gateway address
			sx1272._nodeAddress = DEST_ADDR;
		}
	}
}
