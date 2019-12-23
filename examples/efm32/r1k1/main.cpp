#include "Arduino.h"
#include "pt1000.h"

void setup() {

	pinMode(8, OUTPUT);
	digitalWrite(8, HIGH);

	Serial.setRouteLoc(1);
	Serial.begin(115200);

	//adc.reference(adcRefVDD);
	pt1000_init();
}

void loop() {

	Serial.println("Measure the resistor: ");

	int a6 = adc.read(A6);
	int a7 = adc.read(A7);

	Serial.print("ADC6 = ");
	Serial.print(a6);
	Serial.print(" ADC7 = ");
	Serial.println(a7);

	Serial.print("Rt = ");
	Serial.println( 1100.0 / ((float)a6 / (float)a7 - 1.0), 1 );

	Serial.print("Rt = ");
	Serial.println(pt1000_get_rt());

	delay(2000);  // wait for 2 seconds
}
