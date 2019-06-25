/*
 * SimpleClientGoogleWeatherDHCP
 * gets xml data from http://www.google.com/ig/api?weather=london,uk
 * reads temperature from field:  <temp_f data="66" /> 
 * writes temperature  to analog output port.
 */

#if ARDUINO > 18
#include <SPI.h>         // needed for Arduino versions later than 0018 
#endif

#include <Ethernet.h>
#include "Dhcp.h"  // uses DHCP code from: http://blog.jordanterrell.com/post/Arduino-DHCP-Library-Version-04.aspx 
#include <TextFinder.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte server[] = {209,85,229,104 }; // Google

Client client(server, 80);

TextFinder  finder( client );  

void setup()
{
  Serial.begin(9600);
  if(Dhcp.beginWithDHCP(mac) == 1) {  // begin method returns 1 if successful  
    Serial.println("got IP address, connecting...");
    delay(5000);  
  }
  else {
    Serial.println("unable to acquire ip address!");
    while(true)
       ;  // do nothing
  }
}


void loop()
{
  if (client.connect()) {
    client.println("GET /ig/api?weather=london,uk HTTP/1.0");  // google weather for London
    client.println();
  } 
  else {
    Serial.println(" connection failed");
  } 
  if (client.connected()) {
     // get the feild for temperature in fahrenheit (use field "<temp_c data=\"" for Celcius)
     if(finder.find("<temp_f data=") )
     {      
        int temperature = finder.getValue();
        analogWrite(3, temperature);      // write the temperature to the analog port
        Serial.print("Temperature is ");  // and echo it to the serial port.
        Serial.println(temperature);
     } 
    else
      Serial.print("Could not find temperature field"); 
  }
  else {
    Serial.println("Disconnected"); 
  }
  client.stop();
  client.flush();  
  delay(60000); // wait a minute before next update
}




