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
#include "rtcdriver.h"
#include "em_wdog.h"

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
#define	TX_TIME					200		// 200ms
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

#define	HEARTBEAT_TIME			7100

#define	INIT_RX_INTVAL			120000
#define	INIT_RX_WINDOW			2000

/* Timer used for bringing the system back to EM0 */
RTCDRV_TimerID_t xTimerForWakeUp;

uint32_t rx_intval = INIT_RX_INTVAL;
uint32_t rx_window = INIT_RX_INTVAL + INIT_RX_WINDOW;
uint32_t rx_flag = 0;
uint32_t rx_count = 0;
uint32_t cc_count = 0;

int8_t need_push = 0;
int8_t need_cc = 0;

#define SYS_1S_TICK				(F_CPU)		/* 14MHz, 1Tick = 1/14us, 14000tick = 1000us */

char msg_buf[2][24];

char dev_id[24];
//char pair_id[12] = "11902460803";
char pair_id[12] = "11902041155";

/*
 * Output Mode:
 *
*/
int omode = 1;

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

void change_rxmode()
{
	omode++;
	omode %= 3;

	rx_intval = INIT_RX_INTVAL;
	rx_window = INIT_RX_INTVAL + INIT_RX_WINDOW;

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
	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = wdogPeriod_256k;	/* 256k 1kHz periods should give 256 seconds */

	// Key connected to D0
	pinMode(KEY_PIN, INPUT);
	attachInterrupt(KEY_PIN, change_rxmode, FALLING);

	// dev power ctrl
	pinMode(PWR_CTRL_PIN, OUTPUT);

	/* Initialize RTC timer. */
	RTCDRV_Init();
	RTCDRV_AllocateTimer(&xTimerForWakeUp);

#ifdef DEBUG
	Serial.setRouteLoc(1);
	Serial.begin(115200);
#endif

	/* Start watchdog */
	WDOG_Init(&wInit);

	/* bootup tx */
	need_push = 0x5a;
	need_cc = 0x5a;
}

void task_cc(RTCDRV_TimerID_t id, void *user)
{
	(void)id;
	(void)user;

	WDOG_Feed();

	RTCDRV_StopTimer(xTimerForWakeUp);

	cc_count++;

	if (cc_count >= HEARTBEAT_TIME/rx_intval) {
		need_push = 0x5a;
		cc_count = 0;
	}

	need_cc = 0x5a;
}

void cc_worker()
{
	int i; 
	int e;
	int start = 0, end = 0, rx_ts = 0;

	power_on_dev();

	radio_setup();

	start = millis();

	for (i = 0; i < rx_window/RX_TIME; i++) {

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

			} else if (strcmp(dev_id, pair_id) == 0) {

				rx_ts = millis();

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

				rx_flag = 1;
				break;
			}
		}
	}

here:
	end = millis();

	INFO_S("%s", "RX Time: ");
	INFOLN("%d", (end - start));

	if (0 == rx_flag) {
#if 0
		rx_intval--;
		rx_window += 2;

		if (rx_window == INIT_RX_WINDOW)
			rx_intval = INIT_RX_INTVAL - (end-start)/1000;
#endif

		rx_count++;
	}

	if (1 == rx_flag) {

		rx_intval = INIT_RX_INTVAL - (end - rx_ts);

		rx_window = INIT_RX_WINDOW;

		rx_flag = 0;					// reset flag
		rx_count = 0;
	}

	if (rx_count >= 30) {

		rx_intval = INIT_RX_INTVAL;
		rx_window = INIT_RX_INTVAL + INIT_RX_WINDOW;

		rx_count = 0;
	}

	INFO_S("%s", "window = ");
	INFOLN("%d", rx_window);
	INFO_S("%s", "intval = ");
	INFOLN("%d", rx_intval);
}

void loop(void)
{

	if (0x5a == need_push) {
		// push_stat_data();
		need_push = 0;
	}

	if (0x5a == need_cc) {
		cc_worker();
		need_cc = 0;
	}

	power_off_dev();
	digitalWrite(SX1272_RST, LOW);

	spi_end();

	/*
	 * Enable rtc timer before enter deep sleep
	 * Stop rtc timer after enter task_cc()
	 */
	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypeOneshot, rx_intval, task_cc, NULL);

	EMU_EnterEM2(true);
}
