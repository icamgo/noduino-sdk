
#include "sx126x.h"

//#define TX_ASYNC 				1

#define RF_FREQ			472500000	// Hz center frequency
#define TX_PWR			22			// dBm tx output power

#define	PWR_CTRL_PIN			8	// PC14-D8
#define RF_INT_PIN				3

SX126x lora(SW_CS,		// Pin: SPI CS,PIN06-PB08-D2
	    9,				// Pin: RESET, PIN18-PC15-D9
	    5,				// PIN: Busy,  PIN11-PB14-D5, The RX pin
	    3				// Pin: DIO1,  PIN08-PB11-D3
    );

void radio_init()
{
	lora.init();
	lora.setup_v0(RF_FREQ, TX_PWR);
}

void power_on_dev()
{
	digitalWrite(PWR_CTRL_PIN, HIGH);
}

void power_off_dev()
{
	digitalWrite(PWR_CTRL_PIN, LOW);
}

#ifdef TX_ASYNC
void rx_irq_handler()
{
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);

	uint16_t irq = lora.get_irq_status();

	if (irq & SX126X_IRQ_TX_DONE) {
		lora.clear_tx_active();

	} else if (irq & SX126X_IRQ_TIMEOUT) {
		lora.clear_tx_active();
	}

	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}
#endif

void setup()
{
	Serial.setRouteLoc(1);
	Serial.begin(115200);

	pinMode(PWR_CTRL_PIN, OUTPUT);
	power_on_dev();

	delay(100);

	radio_init();

#ifdef TX_ASYNC
	// RF Interrupt pin
	pinMode(RF_INT_PIN, INPUT);
	attachInterrupt(RF_INT_PIN, rx_irq_handler, RISING);
#endif

	lora.clear_irq_status(SX126X_IRQ_ALL);
}

/* 20430011 */
uint8_t p[36] = {
	0x47, 0x4F, 0x33,
	0x00, 0x00, 0x00, 0x02, 0xCC, 0x79, 0x34, 0xBB,
	0x00, 0xD9, 0x0C, 0x80, 0x00, 0x00, 0x01, 0x63,
	0x00, 0x5F, 0x60, 0x6F, 0xBB, 0x00, 0x00, 0x00, 0x17,
	0x00, 0x00, 0x06, 0xC8,
	0xF3, 0x01, 0x2B, 0x22
};

void loop()
{
	Serial.println("TX testing...");

	int start = 0, end = 0;

	start = millis();

#ifdef TX_ASYNC
	lora.enable_cad();
	int e = lora.send(p, 36, SX126x_TXMODE_ASYNC);
#else
	/*
	 * setup_v0 15:00: 18B - 142ms; 24B - 168ms; 36B - 207ms
	 * setup_v0 17:00: 18B - 123ms; 24B - 147ms; 36B - 184ms
	 */
	lora.enable_cad();
	int e = lora.send(p, 36, SX126x_TXMODE_SYNC);
#endif

	end = millis();

	if (e != 0) {
		if (e == 1)
			Serial.println("TX timeout ...");

		if (e == 2)
			Serial.println("TX failed ...");
	}

	Serial.print("TX time: ");
	Serial.println(end-start);

	delay(6000);
}
