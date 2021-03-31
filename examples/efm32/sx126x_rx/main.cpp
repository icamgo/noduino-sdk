
#include "sx126x.h"

#define TXRX_CH			472500000	// Hz center frequency
#define TX_PWR			22			// dBm tx output power

#define	PWR_CTRL_PIN			8	// PC14-D8
#define RX_INT_PIN				3

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

uint8_t g_buf[64];

void rx_irq_handler()
{
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);

#if 0
	int8_t e = 0;

	if (!e) {

	}
	

#endif

	//int len = lora.rx(g_buf, 64);
	//Serial.print(len);

	//uint16_t irq = lora.get_irq_status();
	Serial.println("rx");
	//lora.clear_irq_status(SX126X_IRQ_RX_DONE);

	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

void setup()
{
	Serial.setRouteLoc(1);
	Serial.begin(115200);

	// RF RX Interrupt pin
	pinMode(RX_INT_PIN, INPUT);
	attachInterrupt(RX_INT_PIN, rx_irq_handler, RISING);

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

	Serial.println(irq, HEX);

	if (irq & SX126X_IRQ_RX_DONE) {
		int len = lora.rx(g_buf, 64);
		Serial.print(len);
		Serial.println(" Bytes RXed");
		lora.clear_irq_status(SX126X_IRQ_RX_DONE);
	}

	lora.clear_irq_status(0xFFFF);

	#if 0
	while ((!(irq & SX126X_IRQ_TX_DONE))
		   && (!(irq & SX126X_IRQ_TIMEOUT))) {
		irq = get_irq_status();
	}
	#endif

	//lora.wakeup();

	lora.enter_rx();

	delay(3000);
}
