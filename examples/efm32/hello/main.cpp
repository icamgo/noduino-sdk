#include "Arduino.h"

void setup()
{
	Serial.setRouteLoc(1);
	Serial.begin(2400);
}

void loop()
{
	Serial.println("Hello, world!");
	delay(2300);
}
