
#include <SX126x.h>

#define RF_FREQUENCY                                433000000 // Hz  center frequency
#define TX_OUTPUT_POWER                             22        // dBm tx output power
#define LORA_BANDWIDTH                              4         // bandwidth=125khz  0:250kHZ,1:125kHZ,2:62kHZ,3:20kHZ.... look for radio line 392                                                               
#define LORA_SPREADING_FACTOR                       7        // spreading factor=11 [SF5..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]

#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false     // variable data payload
#define LORA_IQ_INVERSION_ON                        false
#define LORA_PAYLOADLENGTH                          0         // 0: variable receive length 
                                                              // 1..255 payloadlength
                                                              
uint8_t* pRxData = new uint8_t[128];
int16_t rv = 0;

SX126x  lora(PD5,               //Port-Pin Output: SPI select
             PD6,               //Port-Pin Output: Reset 
             PD7,               //Port-Pin Input:  Busy
             PB0                //Port-Pin Input:  Interrupt DIO1 
             );

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  delay(500);

  lora.begin(SX126X_PACKET_TYPE_LORA,   //LoRa or FSK, FSK currently not supported
             433000000,                 //frequency in Hz
             -3);                       //tx power in dBm
  
  lora.LoRaConfig(LORA_SPREADING_FACTOR, 
                    LORA_BANDWIDTH, 
                    LORA_CODINGRATE, 
                    LORA_PREAMBLE_LENGTH, 
                    LORA_PAYLOADLENGTH, 
                    false,              //crcOn  
                    false);             //invertIrq
   

}

void loop() 
{
  uint8_t rxLen = lora.Receive(pRxData, 128);
  if ( rxLen > 0 )
  { 
    uint8_t rssi, snr;
    Serial.println(pRxData[0], DEC);
    lora.ReceiveStatus(&rssi, &snr);
    Serial.print("snr: ");
    Serial.println(snr, DEC);
    Serial.print("rssi: ");
    Serial.println(rssi, DEC);
  }
  delay(1000);
}
