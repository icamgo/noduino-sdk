/*
 * WebServerParsing
 *
 * Respond to requests in the URL to change digital and analog output ports
 * show the number of ports changed and the value of the analog input pins.
 *
 * the following url turns digital pin 2 on and 3 off, analog output on pin 9 is set to 128, pin 11 to 255 
 * only onen parm http://192.168.1.177/2
 *
 * http://192.168.1.177/?pinD2=1&pinD3=0&pinA9=128&pinA11=255
 */
 
#if ARDUINO > 18
#include <SPI.h>         // needed for Arduino versions later than 0018 
#endif

#include <Ethernet.h>
#include <TextFinder.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte ip[] = { 192,168,1,177 };

Server server(80);

void setup()
{
  Serial.begin(9600);
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.println("ready");
}

void loop()
{
  Client client = server.available();
  if (client) {
    TextFinder  finder(client );  
    // an http request ends with a blank line
    //    boolean current_line_is_blank = true;
    while (client.connected()) {      
      if (client.available()) {          
        int digitalRequests = 0;  // counters to show the number of pin change requests
        int analogRequests = 0;
        if( finder.find("GET /") ) {            
          // find tokens starting with "pin" and stop on the first blank line  
          while(finder.findUntil("pin", "\n\r")){  
            char type = client.read(); // D or A
            int pin = finder.getValue();
            int val = finder.getValue();
            if( type == 'D') {                 
              Serial.print("Digital pin ");
              pinMode(pin, OUTPUT);
              digitalWrite(pin, val);
              digitalRequests++;
            }
            else if( type == 'A'){                  
              Serial.print("Analog pin ");  
              analogWrite(pin, val); 
              analogRequests++;
            }
            else {  
              Serial.print("Unexpected type "); 
              Serial.print(type);                
            }               
            Serial.print(pin);
            Serial.print("=");
            Serial.println(val);
          }                         
        }        
        Serial.println();
        
        // when we get here the findUntil has detected the blank line (a lf followed by cr)
        // so the http request has ended and we can send a reply
        // send a standard http response header
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println();

        // output the number of pins handled by the request
        client.print(digitalRequests);
        client.print(" digital pin(s) written");
        client.println("<br />");
        client.print(analogRequests);
        client.print(" analog pin(s) written");
        client.println("<br />");
        client.println("<br />");

    
        // output the value of each analog input pin
        for (int i = 0; i < 6; i++) {
          client.print("analog input ");
          client.print(i);
          client.print(" is ");
          client.print(analogRead(i));
          client.println("<br />");
        }
        break;        
      } 
    }
    // give the web browser time to receive the data
    delay(1);
    client.stop();
  }
}

