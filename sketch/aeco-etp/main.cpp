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
#define	RX_INT_PIN				3		/* PIN8_PB11_D3 */

#define SDA_PIN					12		/* PIN23_PE12 */
#define SCL_PIN					13		/* PIN24_PE13 */

#define	CH1_PIN					SCL_PIN
#define	CH2_PIN					SDA_PIN

#define SYNCWORD_DEFAULT		0x12
#define SYNCWORD_LORAWAN		0x34
#define SYNCWORD_ABC			0x55

//#define	ENABLE_CAD				1

#ifdef CONFIG_V0

#define RECEIVE_ALL
#define TXRX_CH					CH_01_472
#define RX_TIME					180
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

/* Timer used for bringing the system back to EM0 */
RTCDRV_TimerID_t xTimerForWakeUp;

uint32_t ck_intval = 20;			// 20s
uint32_t rx_flag = 0;
uint32_t rx_count = 0;
uint32_t ck_count = 0;

int8_t need_push = 0;

#define SYS_1S_TICK				(F_CPU)		/* 14MHz, 1Tick = 1/14us, 14000tick = 1000us */

char msg_buf[2][24];

char rx_devid[24];
char my_devid[12];

/*
 * Working Mode:
 *
*/
int wmode = 1;

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
	//sx1272.setSyncWord(SYNCWORD_ABC);
#else
	sx1272.sx1278_qsetup(TXRX_CH, MAX_DBM);
	sx1272._nodeAddress = DEST_ADDR;
#endif

#ifdef ENABLE_CAD
	sx1272._enableCarrierSense = true;
#endif
}

char *uint64_to_str(uint64_t n, char *buf)
{
	char *dest = buf;

	dest += 20;
	*dest-- = 0;
	while (n) {
		*dest-- = (n % 10) + '0';
		n /= 10;
	}

	strcpy(buf, dest+1);

	return dest + 1;
}

char *decode_devid(uint8_t *pkt)
{
	int a = 0, b = 0;

	uint64_t devid = 0UL;

	for (a = 3; a < 11; a++, b++) {

		*(((uint8_t *)&devid) + 7 - b) = pkt[a];
	}

	return uint64_to_str(devid, rx_devid);
}

uint64_t get_my_devid()
{
	uint64_t *p;

	p = (uint64_t *)0x0FE00008;

	return *p;
}

char *decode_my_devid()
{
	uint64_t myid = get_my_devid();

	return uint64_to_str(myid, my_devid);	
}

void change_mode()
{
	wmode++;
	wmode %= 3;

	INFO("%s", "wmode: ");
	INFOLN("%d", wmode);
}

void power_on_dev()
{
	digitalWrite(PWR_CTRL_PIN, HIGH);
}

void power_off_dev()
{
	digitalWrite(PWR_CTRL_PIN, LOW);
}

void ch1_on()
{
	digitalWrite(CH1_PIN, HIGH);
}

void ch1_off()
{
	digitalWrite(CH1_PIN, LOW);
}

void ch2_on()
{
	digitalWrite(CH2_PIN, HIGH);
}

void ch2_off()
{
	digitalWrite(CH2_PIN, LOW);
}

void ch3_on()
{
	//digitalWrite(CH3_PIN, HIGH);
}

void ch3_off()
{
	//digitalWrite(CH3_PIN, LOW);
}

void level1()
{
	ch2_off();
	ch3_off();
	ch1_on();
}

void level2()
{
	ch1_off();
	ch3_off();
	ch2_on();
}

void level3()
{
	ch1_off();
	ch2_off();
	ch3_on();
}

void task_ck(RTCDRV_TimerID_t id, void *user)
{
	(void)id;
	(void)user;

	WDOG_Feed();

	ck_count++;

	if (ck_count >= HEARTBEAT_TIME/ck_intval) {
		need_push = 0x5a;
		ck_count = 0;
	}
}

void rx_irq_handler()
{
	INFOLN("%s", "new rx pkt...");
}

void setup()
{
	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = wdogPeriod_32k;	/* 32k 1kHz periods should give 32 seconds */

	// Key connected to D0
	pinMode(KEY_PIN, INPUT);
	//attachInterrupt(KEY_PIN, change_mode, FALLING);

	// RF RX Interrupt pin
	pinMode(RX_INT_PIN, INPUT);
	attachInterrupt(RX_INT_PIN, rx_irq_handler, FALLING);

	// dev power ctrl
	pinMode(PWR_CTRL_PIN, OUTPUT);

	// relay ctrl
	pinMode(CH1_PIN, OUTPUT);
	pinMode(CH2_PIN, OUTPUT);

	ch1_off();
	ch2_off();
	ch3_off();

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

	/* fill the my_devid */
	decode_my_devid();

	power_on_dev();

	radio_setup();

	/*
	 * Enable rtc timer 
	 */
	RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypePeriodic, ck_intval*1000, task_ck, NULL);
}

uint16_t get_crc(uint8_t *pp, int len)
{
	int i;
	uint16_t hh = 0;

	for (i = 0; i < len; i++) {
		hh += pp[i];
	}
	return hh;
}

void rx_worker()
{
	int i; 
	int e;
	int start = 0, end = 0, rx_ts = 0;

	start = millis();

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

		// get the devid of the rx pkt
		decode_devid(rx_pkt);

		if (strcmp(rx_devid, "") == 0) {
			// lora module unexpected error
			sx1272.reset();

			INFO_S("%s", "Resetting lora module\n");

		} else if (strcmp(rx_devid, my_devid) == 0) {

			rx_ts = millis();

			switch(rx_pkt[11]) {
				case 0x10:
					// Level 1
					INFOLN("%s", "Set to Level 1");
					level2();
					delay(2000);
					level1();
					break;
				case 0x20:
					// Level 2
					INFOLN("%s", "Set to Level 2");
					level2();
					break;
				case 0x30:
					// Level 3
					INFOLN("%s", "Set to Level 3");
					level3();
					break;
				default:
					// all off
					INFOLN("%s", "Set to Off!");
					ch1_off();
					ch2_off();
					ch3_off();
			}

#if 0
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
#endif
		}
	}
}

void loop(void)
{
	if (0x5a == need_push) {
		// push_stat_data();
		need_push = 0;
	}

	rx_worker();

}
