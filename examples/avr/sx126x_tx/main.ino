
#include <SX126x.h>

#define RF_FREQ			472500000	// Hz center frequency
#define TX_PWR			14			// dBm tx output power
#define LORA_BW			6
#define LORA_SF			10			
#define LORA_CR			2
#define	CRC				true

#define LORA_PREAMBLE_LEN				12	// Same for Tx and Rx
#define LORA_PAYLOAD_LEN				0	// 0: variable receive length
							      			// 1..255 payloadlength

SX126x lora(10,		// Pin: SPI CS
	    5,			// Pin: RESET
	    2,			// PIN: Busy
	    3			// Pin: DIO1 
    );

void radio_init()
{
	lora.begin(SX126X_PACKET_TYPE_LORA, RF_FREQ, TX_PWR);
	lora.LoRaConfig(LORA_SF, LORA_BW, LORA_CR, LORA_PREAMBLE_LEN, LORA_PAYLOAD_LEN, CRC, false);
}

void setup()
{
	Serial.begin(115200);

	radio_init();
}

uint8_t i;

uint8_t p[24] = {
	0x34, 0x00, 0x12,
	0x22, 0x00, 0x00, 0x02, 0xCB, 0x63, 0x09, 0xE7,
	0x0B, 0xB8, 0x0E, 0x45, 0x03, 0xC4,
	0xD8, 0x05, 0x9E, 0x00, 0x00, 0x00, 0x00
};

//47 4F 33 00 00 00 02 CB 63 09 E7 0B B8 0E 45 02 C4 D8 05 9D 22 A5 4A A3

void loop()
{
	Serial.println("TX PKT...");

	lora.Send(p, 20, SX126x_TXMODE_ASYNC);

	i++;

	delay(3000);
}
