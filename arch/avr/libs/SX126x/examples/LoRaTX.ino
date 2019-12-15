
#include <SX126x.h>

#define RF_FREQ			470000000 // Hz  center frequency
#define TX_PWR			22        // dBm tx output power
#define LORA_BW			4         // bandwidth=125khz  0:250kHZ,1:125kHZ,2:62kHZ,3:20kHZ.... look for radio line 392                                                               
#define LORA_SF			7			// spreading factor=11 [SF5..SF12]
#define LORA_CR			1			// [1: 4/5,
									//  2: 4/6,
									//  3: 4/7,
									//  4: 4/8]

#define LORA_PREAMBLE_LEN				8		// Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT				0		// Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON		false	// variable data payload
#define LORA_IQ_INVERSION_ON			false
#define LORA_PAYLOAD_LEN				0		// 0: variable receive length 
                                                              // 1..255 payloadlength

SX126x lora(PD5,               //Port-Pin Output: SPI select
             PD6,               //Port-Pin Output: Reset 
             PD7,               //Port-Pin Input:  Busy
             PB0                //Port-Pin Input:  Interrupt DIO1 
             );


void setup() 
{
	// put your setup code here, to run once:
	Serial.begin(9600);

	delay(500);

	lora.begin(SX126X_PACKET_TYPE_LORA, RF_FREQ, TX_PWR);

	lora.LoRaConfig(LORA_SF, 
					LORA_BW, 
					LORA_CR, 
					LORA_PREAMBLE_LEN, 
					LORA_PAYLOAD_LEN, 
					false,              //crcOn  
					false);             //invertIrq
}

uint8_t i;

void loop() 
{
	lora.Send(&i, 1,  SX126x_TXMODE_SYNC);
	i++;
	delay(1000);
}
