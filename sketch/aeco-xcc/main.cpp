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

#include <stdio.h>
#include <stdint.h>

#include "softi2c.h"
#include "pcf8563.h"

#include "sx126x.h"
#include "crypto.h"

#include "tx_ctrl.h"
#include "circ_buf.h"


#define TXRX_CH			472500000	// Hz center frequency
#define TX_PWR			22			// dBm tx output power

#define	PWR_CTRL_PIN			8	// PC14-D8
#define RF_INT_PIN				3

SX126x lora(2,		// Pin: SPI CS,PIN06-PB08-D2
	    9,				// Pin: RESET, PIN18-PC15-D9
	    5,				// PIN: Busy,  PIN11-PB14-D5, The RX pin
	    3				// Pin: DIO1,  PIN08-PB11-D3
    );

struct circ_buf g_cbuf __attribute__((aligned(4)));
struct ctrl_fifo g_cfifo __attribute__((aligned(4)));

void power_on_dev()
{
	digitalWrite(PWR_CTRL_PIN, HIGH);
}

void power_off_dev()
{
	digitalWrite(PWR_CTRL_PIN, LOW);
}

bool check_crc(uint8_t *p, int plen)
{
	int i, len = 0;
	uint16_t hh = 0, sum = 0;

	len = plen - 6;
	sum = p[len] << 8 | p[len+1];

	for (i = 0; i < len; i++) {
		hh += p[i];
	}

	if (hh == sum)
		return true;
	else
		return false;
}

uint32_t get_dev_type(uint8_t *p)
{
	uint64_t devid = 0UL;
	uint8_t *pd = (uint8_t *) &devid;

	for(int i = 0; i < 8; i++) {
		pd[7-i] = p[3+i];
	}

	uint64_t temp = devid / 1000000;
	uint32_t tt = (uint32_t)(temp / 100);
	return (uint32_t)(temp - tt * 100);
}

bool is_my_pkt(uint8_t *p, int len)
{
	/*
	 * 31: 17
	 * 32: 19
	 * 33: 20
	*/
	if (len < 36) return false;

	if (p[0] != 0x47 || p[1] != 0x4F) {

		// invalide pkt header
		return false;
	}

	// check the devid quickly in interrupt contex
	if (p[3] != 0 || p[4] != 0 || p[5] != 0 || p[6] > 0x17) {

		// invalid devid. max_id = 0x17 ff ff ff ff (1_030.79.21.5103)
		return false;
	}

	if (p[2] < 0x32 || p[2] > 0x36) {

		// support only the 0x33/34/35/36 version
		return false;
	}

	if (p[len-9] & 0x10) {
		// new cc relayed pkt
		return false;
	}

	// pkt[27] = 0b100, set bit 2, HTimer rpt pkt
	if (20 != get_dev_type(p) && 0 == (p[len-9] & 0x4)) {
		// It's not the HTimer rpt pkt
		return false;
	}

#if 0
	if (check_crc(p, len) == false) {

		return false;
	}

	if (check_pkt_mic(p, len) == 0) {
		return false;		
	}
#endif

	return true;
}

void rx_irq_handler()
{
	int len = 0, rssi = 0;
	uint8_t p[48];

	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);

	len = lora.get_rx_pkt(p, 48);
	rssi = lora.get_pkt_rssi();

	if (len > PKT_LEN || len < 0) goto irq_out;

	if (is_my_pkt(p, len)) {
		push_pkt(&g_cbuf, p, rssi, len);
	}

irq_out:

	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

void hex_pkt(uint8_t *p, int rssi, int plen)
{
	int a = 0;

	for (; a < plen; a++) {

		if ((uint8_t) p[a] < 16)
			Serial.print("0");

		Serial.print((uint8_t) p[a], HEX);
		Serial.print(" ");
	}

	Serial.print("/");
	Serial.print(rssi);
	Serial.print("/");
	Serial.print(plen);

	Serial.print("/");
	Serial.println(check_pkt_mic(p, plen));
}

void setup()
{
	crypto_init();

	Serial.setRouteLoc(1);
	Serial.begin(115200);

	// RF Interrupt pin
	pinMode(RF_INT_PIN, INPUT);
	attachInterrupt(RF_INT_PIN, rx_irq_handler, RISING);

	pinMode(PWR_CTRL_PIN, OUTPUT);
	power_on_dev();

	delay(100);

	lora.reset();
	lora.init();
	lora.setup_v0(TXRX_CH, TX_PWR);
	lora.enter_rx();

	Serial.println("RX testing...");
}

struct pkt d;

void loop()
{
	noInterrupts();
	int ret = get_pkt(&g_cbuf, &d);
	interrupts();

	if (ret != 0) {
		// there is no pkt
		//Serial.print(".");
	} else {
		hex_pkt(d.data, d.rssi, d.plen);
	}

	//delay(5000);
}
