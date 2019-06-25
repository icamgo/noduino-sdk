/*
 * SimpleClientTextFinder.pde
 *
 */
 
#if ARDUINO > 18
#include <SPI.h>         // needed for Arduino versions later than 0018 
#endif

#include <Ethernet.h>
#include <TextFinder.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte ip[] = { 192,168,1,177 };
byte gateway[] ={ 192, 168, 1, 1 };
byte subnet[] ={ 255, 255, 255, 0 };

byte server[] = {209,85,229,104 }; // Google

Client client(server, 80);
TextFinder finder( client);

long hits; // the number of google hits

void setup()
{
  Ethernet.begin(mac, ip, gateway, subnet); 
  Serial.begin(9600);
  Serial.println("connecting...");  
  delay(2000);  
}

void loop()
{
  if (client.connect()) {
    Serial.print("connected... ");
    client.println("GET /search?q=arduino HTTP/1.0");
    client.println();
  } else {
    Serial.println("connection failed");
  }
  if (client.connected()) {
    hits = 0; // reset hit count
    if(finder.find("Results <b>")){    
      if(finder.find("of")){     
         hits = finder.getValue(',');
         Serial.print(hits);
         Serial.println(" hits");         
      }
    }
    if(hits == 0){
      Serial.println("No Google hits "); 
    }
    delay(10000); // check again in 10 seconds
  }
  else {
    Serial.println();
    Serial.println("not connected");
    delay(1000); 
  } 
  client.stop();

}
