#include <DES.h>

DES des;

void setup() {
  Serial.begin(9600);
  Serial.println("Hello!");
}

void loop() {
  desTest();
  tdesTest();
  delay(2000);
}

void desTest()
{
  byte out[8];
  byte in[] = { 1, 2, 3, 4, 5, 6, 7, 8 };
  byte key[] = { 0x3b, 0x38, 0x98, 0x37, 0x15, 0x20, 0xf7, 0x5e };
  
  Serial.println();
  Serial.println("========= DES test ==========");
  
  //encrypt
  Serial.print("Encrypt...");
  long time = millis();
  des.encrypt(out, in, key);
  time = millis() - time;
  Serial.print("done. (");
  Serial.print(time);
  Serial.println(" ms)");
  printArray(out);
  
  //decrypt
  for (int i = 0; i < 8; i++)
  {
    in[i] = out[i];
  }
  Serial.print("Decrypt...");
  time = millis();
  des.decrypt(out, in, key);
  time = millis() - time;
  Serial.print("done. (");
  Serial.print(time);
  Serial.println(" ms)");
  printArray(out);
}

void tdesTest()
{
  byte out[8];
  byte in[] = { 1, 2, 3, 4, 5, 6, 7, 8 };
  byte key[] = { 
                  0x3b, 0x38, 0x98, 0x37, 0x15, 0x20, 0xf7, 0x5e, // key A
                  0x92, 0x2f, 0xb5, 0x10, 0xc7, 0x1f, 0x43, 0x6e, // key B
                  0x3b, 0x38, 0x98, 0x37, 0x15, 0x20, 0xf7, 0x5e, // key C (in this case A)
                };
  
  Serial.println();
  Serial.println("====== Triple-DES test ======");
  
  //encrypt
  Serial.print("Encrypt...");
  long time = millis();
  des.tripleEncrypt(out, in, key);
  time = millis() - time;
  Serial.print("done. (");
  Serial.print(time);
  Serial.println(" ms)");
  printArray(out);
  
  //decrypt
  for (int i = 0; i < 8; i++)
  {
    in[i] = out[i];
  }
  Serial.print("Decrypt...");
  time = millis();
  des.tripleDecrypt(out, in, key);
  time = millis() - time;
  Serial.print("done. (");
  Serial.print(time);
  Serial.println(" ms)");
  printArray(out);
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
