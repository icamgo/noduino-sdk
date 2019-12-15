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
	Serial.begin(9600);

	adc.reference(adcRef2V5);
	//adc.reference(adcRefVDD);
}

void loop() {

	Serial.println("Energy monitor testing...");
	Serial.print("Clock Freq = ");
	Serial.println(CMU_ClockFreqGet(cmuClock_CORE));

	Serial.print("ADC differential ch6 ch7 read:");
	Serial.println(adc.read(A6, A7));
#if 0
	Serial.print("ADC6 = ");
	Serial.println(adc.read(A6));
	Serial.print("ADC7 = ");
	Serial.println(adc.read(A7));
	Serial.print("Vbat = ");
	Serial.println(adc.readVbat(), 3);
#endif

	delay(2000);  // wait for 2 seconds
}
