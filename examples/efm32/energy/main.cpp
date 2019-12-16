/*
  adcDiffRead.ino  efm32adc differential read demo
     ADC class func list:
         ADC adcinstence         : creat a ADC instance,  adc is predefined
         resolution(res)         :set resolution
         getResolution()         :get resolution
         reference(int ref)      :set reference
         getReference()          :get reference
         read(pin1, pin2)        ;pin2 default is NOT_A_PIN,SingleInput else deffInput mode
         readTemp()              :read internal temperature ADC value;
         temperatureCelsius()    :temperature degree celsius;
         temperatureFahrenheit() :temperature degree fahrenheit;

*/

void setup() {

	pinMode(8, OUTPUT);
	digitalWrite(8, HIGH);

	Serial.setRouteLoc(1);
	Serial.begin(115200);

	adc.reference(adcRef1V25);
}

void loop() {

	Serial.println("Energy monitor testing...");
	Serial.print("Clock Freq = ");
	Serial.println(CMU_ClockFreqGet(cmuClock_CORE));
#if 0
	Serial.print("ADC6 = ");
	Serial.println(adc.read(A6));
	Serial.print("ADC7 = ");
	Serial.println(adc.read(A7));
	Serial.print("Vbat = ");
	Serial.println(adc.readVbat(), 3);
#endif

	int ad = adc.read(A6, A7);
	Serial.print("ADC differential ch6 ch7 read:");
	Serial.println(ad);

	Serial.print("The consumption current (mA): ");
	Serial.println(1250.0*ad/2.0/2048.0/0.7, 2);

	delay(2000);  // wait for 2 seconds
}
