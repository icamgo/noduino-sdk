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

	Serial.setRouteLoc(1);
	Serial.begin(115200);

	adc.reference(INTERNAL2V5); /* setup ADC reference INTERNAL1V25/INTERNAL2V5/INTERNAL3V3*/
}

void loop() {

	Serial.print("ADC differential ch6 ch7 read:");
	Serial.println(adc.read(A6, A7));

	delay(2000);  // wait for 2 seconds

}
