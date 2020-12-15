
#include <SX126x.h>

//#define RF_FREQ			472500000	// Hz center frequency
//#define RF_FREQ			470020000	// Hz center frequency
#define RF_FREQ			470000000	// Hz center frequency
#define TX_PWR			22			// dBm tx output power
#define LORA_BW			6
#define LORA_SF			10			
#define LORA_CR			2
#define	CRC				true

#define LORA_PREAMBLE_LEN				8	// Same for Tx and Rx
#define LORA_PAYLOAD_LEN				0	// 0: variable receive length
							      			// 1..255 payloadlength

SX126x lora(10,		// Pin: SPI CS
	    5,			// Pin: RESET
	    2,			// PIN: Busy
	    3			// Pin: DIO1 
    );

void radio_init()
{
	lora.begin(RF_FREQ, TX_PWR);
	lora.lora_config(LORA_SF, LORA_BW, LORA_CR, LORA_PREAMBLE_LEN, LORA_PAYLOAD_LEN, CRC, false);
}

void setup()
{
	Serial.begin(115200);

	radio_init();
}

/* 12020430011 */
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
	Serial.println("TX PKT...");

	lora.send(p, 36, SX126x_TXMODE_SYNC);

	delay(8000);
}
