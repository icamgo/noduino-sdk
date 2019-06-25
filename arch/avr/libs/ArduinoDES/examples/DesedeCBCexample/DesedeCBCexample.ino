#include <DES.h>

#define MESSAGE_LENGTH 16

DES des;

byte key[] =  { 
                0xDE, 0x4F, 0x86, 0xB8, 0x38, 0x94, 0x74, 0x75, // key A
                0x90, 0x13, 0x39, 0xE5, 0x96, 0x36, 0x94, 0x43, // key B
                0xDE, 0x4F, 0x86, 0xB8, 0x38, 0x94, 0x74, 0x75  // key C (in this case A)
              };

void setup() {
  Serial.begin(9600);
  Serial.println("Hello!");
}

void loop() {
  tdesCbcEncipherTest();
  tdesCbcDecipherTest();
  delay(2000);
}

void tdesCbcEncipherTest()
{
  byte out[MESSAGE_LENGTH];
  byte in[] = { 0xA3, 0xA7, 0x2C, 0x1F, 0xEC, 0x9F, 0x1E, 0x8B, 0xBC, 0x56, 0xC3, 0xA8, 0xF9, 0x47, 0x3C, 0xD6 };
  byte iv[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  
  Serial.println();
  Serial.println("====== Triple-DES CBC encipher test ======");
  
  Serial.print("Encrypt...");
  long time = millis();
  
  for (int i = 0; i < MESSAGE_LENGTH; i += 8)
  {
    //CBC algorithm
    //IMPORTANT: THIS WILL MODIFY THE INPUT ARRAY!!!
    for (byte xorIdx = 0; xorIdx < 8; xorIdx++)
    {
      // 0 -> use IV; >0 -> use ciphertext
      if (i == 0)
      {
        in[i + xorIdx] = in[i + xorIdx] ^ iv[xorIdx];
      }
      else
      {
        in[i + xorIdx] = in[i + xorIdx] ^ out[(i - 8) + xorIdx];
      }
    }
    
    des.tripleEncrypt((byte*)out + i, (byte*)in + i, key);
  }
  
  time = millis() - time;
  Serial.print("done. (");
  Serial.print(time);
  Serial.println(" ms)");
  
  for (int i = 0; i < MESSAGE_LENGTH; i += 8)
  {
    printArray((byte*)out + i);
  }
}

void tdesCbcDecipherTest()
{
  byte out[MESSAGE_LENGTH];
  byte in[] = { 0xA3, 0xA7, 0x2C, 0x1F, 0xEC, 0x9F, 0x1E, 0x8B, 0xBC, 0x56, 0xC3, 0xA8, 0xF9, 0x47, 0x3C, 0xD6 };
  byte iv[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  
  Serial.println();
  Serial.println("====== Triple-DES CBC decipher test ======");
  
  Serial.print("Decrypt...");
  long time = millis();
  
  for (int i = 0; i < MESSAGE_LENGTH; i += 8)
  {
    des.tripleDecrypt((byte*)out + i, (byte*)in + i, key);
    
    //CBC algorithm
    for (byte xorIdx = 0; xorIdx < 8; xorIdx++)
    {
      // 0 -> use IV; >0 -> use ciphertext
      if (i == 0)
      {
        out[i + xorIdx] = out[i + xorIdx] ^ iv[xorIdx];
      }
      else
      {
        out[i + xorIdx] = out[i + xorIdx] ^ in[(i - 8) + xorIdx];
      }
    }
  }
  
  time = millis() - time;
  Serial.print("done. (");
  Serial.print(time);
  Serial.println(" ms)");
  
  for (int i = 0; i < MESSAGE_LENGTH; i += 8)
  {
    printArray((byte*)out + i);
  }
}

void printArray(byte output[])
{
  for (int i = 0; i < 8; i++)
  {
    if (output[i] < 0x10)
    {
      Serial.print("0");
    }
    Serial.print(output[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}