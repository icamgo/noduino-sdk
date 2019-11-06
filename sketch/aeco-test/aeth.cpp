void setup() {

	/* setup alternate location default is 0# */  
	/* set to 1#. the serial port's Alternate LOCATION(see datasheet)*/
	Serial.setRouteLoc(1);
	Serial.begin(115200);
  
}

int id = 0;

void loop() {

	if (Serial.available() > 0) {

		id = Serial.read();

		Serial.print("I received: ");
		Serial.println(id, DEC);
	}
}
