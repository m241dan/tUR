//Dumbass fucking stupid sketch to replace our RS232 to USB reader.
//Reads in serial data coming out of vac chamber and passes it over USB to computer.
// 23 Feb 2018 - Jimmy :(

#include "String.h"
String incomingString = "aaa";

void setup() {
  Serial.begin(9600);

}

void loop() {
  //MEGA-ONLY SYNTAX; change for UNO. -JEA
  incomingString = Serial1.read();
  Serial.println(incomingString);
  delay(500);

}
