
#include "sx126x.h"
#include "crypto.h"


#define TXRX_CH			472500000	// Hz center frequency
#define TX_PWR			22			// dBm tx output power

#define	PWR_CTRL_PIN			8	// PC14-D8
#define RF_INT_PIN				3

SX126x lora(2,		// Pin: SPI CS,PIN06-PB08-D2
	    9,				// Pin: RESET, PIN18-PC15-D9
	    5,				// PIN: Busy,  PIN11-PB14-D5, The RX pin
	    3				// Pin: DIO1,  PIN08-PB11-D3
    );

void power_on_dev()
{
	digitalWrite(PWR_CTRL_PIN, HIGH);
}

void power_off_dev()
{
	digitalWrite(PWR_CTRL_PIN, LOW);
}

uint8_t g_buf[256];

void rx_irq_handler()
{
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);

	uint16_t irq = lora.get_irq_status();
	Serial.print(irq, HEX);
	Serial.println(" ,rx");

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

void loop()
{
	uint16_t irq = 0;
	irq = lora.get_irq_status();

	//Serial.println(irq, HEX);

	if (irq & SX126X_IRQ_RX_DONE) {

		int len = lora.rx(g_buf, 48);		// clear the irq flag in this func
		int rssi = lora.get_pkt_rssi();

		hex_pkt(g_buf, rssi, len);

		if (irq & SX126X_IRQ_CRC_ERR) {
			Serial.println("CRC error");
			lora.clear_irq_status(SX126X_IRQ_CRC_ERR);
		}

	} else if (irq & SX126X_IRQ_TIMEOUT) {

		lora.clear_irq_status(SX126X_IRQ_TIMEOUT);
	}

	//delay(5000);
}
