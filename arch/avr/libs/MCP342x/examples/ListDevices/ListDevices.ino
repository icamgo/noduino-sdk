#include <Wire.h>

void setup(void)
{
  Serial.begin(9600);
  Wire.begin();
  Serial.println("The MCP3422 and MCP3426 use I2C address 0x68, all other devices can be");
  Serial.println("configured to use any address in the range 0x68 - 0x6F (inclusive).");
  Serial.println("Be aware that the DS1307 uses address 0x68.");
  Serial.println();
  
  for (uint8_t add = 0X0; add < 0X80; add++) {
    //Serial.print("Trying ");
    //Serial.println(add);
    Wire.requestFrom(add, (uint8_t)1);
    if (Wire.available()) {
      Serial.print("Found device at: 0x");
      Serial.println(add, HEX);
    }
  }
  Serial.println("Done");
}

void loop(void)
{
  ;
}
