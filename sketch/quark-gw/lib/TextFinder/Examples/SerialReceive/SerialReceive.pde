/*
 * SerialReceiveMultipleFields with textFinder
 * This code expects a message in the format: H 12,-345,678!
 */

#include <TextFinder.h>

TextFinder  finder(Serial);  
const int NUMBER_OF_FIELDS = 3; // how many comma seperated fields we expect                                           
int values[NUMBER_OF_FIELDS];   // array holding values for all the fields

void setup() 
{ 
  Serial.begin(9600); // Initialize serial port to send and receive at 9600 baud
} 

void loop()
{
  int fieldIndex = 0;            // the current field being received
  finder.find("H");   
  while(fieldIndex < NUMBER_OF_FIELDS)
    values[fieldIndex++] = finder.getValue();      
  Serial.println("All fields received:");
  for(fieldIndex=0; fieldIndex < NUMBER_OF_FIELDS; fieldIndex++)
    Serial.println(values[fieldIndex]);               
}



