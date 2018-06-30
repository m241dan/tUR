
#include <Wire.h>
#include "ram_funcs.h"

void setup() 
{
  Wire.begin( );                  // Join i2c bus with blank I2C address to name itself master and rule with an iron fist
  Wire.onReceive(receiveEvent);   // Register event
  Serial.begin(9600);             // Start serial for output
}

void loop() 
{
  Serial.println("Requesting BusyBox packet... \t");
  Wire.requestFrom(I2CADDRESS_BBOX,    sizeof(bbox_packet));
  //Wire.requestFrom(I2CADDRESS_AMBIENT, sizeof(ambient_packet));
  //Wire.requestFrom(I2CADDRESS_PI_CAM,  sizeof(**CHANGEME_packet));
  //Wire.requestFrom(I2CADDRESS_PI_ARM,  sizeof(**CHANGEME_packet));
  while (Wire.available())   // (slave may send less than requested)
  { 
    char c = Wire.read();    // receive a byte as character
    Serial.print(c);         // print the character
  }
  delay(500);
}



// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) 
{
  while (1 < Wire.available()) 
  {                          // loop through all but the last
    char c = Wire.read();    // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire.read();       // receive byte as an integer
  Serial.println(x);         // print the integer
}
