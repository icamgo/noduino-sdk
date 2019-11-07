
void setup()
{
	Serial.setRouteLoc(1); /*set to 1#. the serial port's Alternate LOCATION(see datasheet)*/
	Serial.begin(115200);
	//  adc.reference(INTERNAL2V5); /* setup ADC reference INTERNAL1V25/INTERNAL2V5/INTERNAL3V3*/
}

void loop()
{
	// int tempval = adc.readTemp();
	Serial.print("Read internal temperature :");

	float temp = adc.temperatureFahrenheit();

	Serial.print(temp);
	Serial.print("F (");

	temp = adc.temperatureCelsius();

	Serial.print(temp);
	Serial.println("C)");

	delay(2000);                  // wait for 2 seconds
}
