#include "Arduino.h"

void setup()
{
	Serial.setRouteLoc(1);
	Serial.begin(115200);
}

void loop()
{
	Serial.println("Hello, world!");
	delay(2300);
}
