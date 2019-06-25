#include <Wire.h>
#include <MCP342x.h>

/* Demonstrate the use of read() and convert(). If read() is called
 * immediately after convert then the conversion will not have
 * completed. Two approaches to avoid this problem are possible, use
 * delay() or similar to wait a fixed amount of time, or to
 * periodically read the device and check the config result.
 */


// 0x68 is the default address for all MCP342x devices
uint8_t address = 0x6E;
MCP342x adc = MCP342x(address);

// Configuration settings
MCP342x::Config config(MCP342x::channel1, MCP342x::oneShot,
		       MCP342x::resolution18, MCP342x::gain1);

// Configuration/status read back from the ADC
MCP342x::Config status;

// Inidicate if a new conversion should be started
bool startConversion = false;


// LED details
#ifdef LED_BUILTIN
int led = LED_BUILTIN;
#else
int led = 13;
#endif
bool ledLevel = false;



void setup(void)
{
  Serial.begin(9600);
  Wire.begin();

  // Enable power for MCP342x (needed for FL100 shield only)
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  
  pinMode(led, OUTPUT);
    
  // Reset devices
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle
  
  // Check device present
  Wire.requestFrom(address, (uint8_t)1);
  if (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(address, HEX);
    while (1)
      ;
  }

  // First time loop() is called start a conversion
  startConversion = true;
}

unsigned long lastLedFlash = 0;
void loop(void)
{
  long value = 0;
  uint8_t err;

  if (startConversion) {
    Serial.println("Convert");
    err = adc.convert(config);
    if (err) {
      Serial.print("Convert error: ");
      Serial.println(err);
    }
    startConversion = false;
  }
  
  err = adc.read(value, status);
  if (!err && status.isReady()) { 
    // For debugging purposes print the return value.
    Serial.print("Value: ");
    Serial.println(value);
    Serial.print("Config: 0x");
    Serial.println((int)config, HEX);
    Serial.print("Convert error: ");
    Serial.println(err);
    startConversion = true;
  }

  // Do other stuff here, such as flash an LED
  if (millis() - lastLedFlash > 50) {
    ledLevel = !ledLevel;
    digitalWrite(led, ledLevel);
    lastLedFlash = millis();
  }
    
}

