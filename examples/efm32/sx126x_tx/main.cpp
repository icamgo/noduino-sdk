
#include "SX126x.h"

//#define RF_FREQ			470000000	// Hz center frequency
#define RF_FREQ			472500000
#define TX_PWR			14				// dBm tx output power
#define LORA_BW			6
#define LORA_SF			10			
#define LORA_CR			2
#define	CRC				true

#define LORA_PREAMBLE_LEN				6	// 8
#define LORA_PAYLOAD_LEN				0	// 0: variable receive length
							      			// 1..255 payloadlength

SX126x lora(SW_CS,		// Pin: SPI CS,PIN06-PB08-D2
	    9,				// Pin: RESET, PIN18-PC15-D9
	    5,				// PIN: Busy,  PIN11-PB14-D5
	    3				// Pin: DIO1,  PIN08-PB11-D3
    );

void radio_init()
{
	lora.begin(RF_FREQ, TX_PWR);
	lora.lora_config(LORA_SF, LORA_BW, LORA_CR, LORA_PREAMBLE_LEN, LORA_PAYLOAD_LEN, CRC, false);
}

#define	PWR_CTRL_PIN			8

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
	//Serial.begin(115200);

	pinMode(PWR_CTRL_PIN, OUTPUT);
	power_on_dev();

	radio_init();
}

uint8_t p[36] = {
	0x47, 0x4F, 0x33,
	0x00, 0x00, 0x00, 0x02, 0xCC, 0x79, 0x34, 0xBB,
	0x00, 0xD9, 0x0C, 0x80, 0x00, 0x00, 0x01, 0x63,
	0x00, 0x5F, 0x60, 0x6F, 0xBB, 0x00, 0x00, 0x00, 0x17,
	0x00, 0x00, 0x06, 0xC8,
	0xF3, 0x01, 0x2B, 0x21
};

//47 4F 33 00 00 00 02 CB 63 09 E7 0B B8 0E 45 02 C4 D8 05 9D 22 A5 4A A3

void loop()
{
	lora.send(p, 36, SX126x_TXMODE_SYNC);

	delay(3000);
}
