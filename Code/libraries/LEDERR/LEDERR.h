#ifndef LEDERR_H
#define LEDERR_H

#include <Arduino.h>

class LEDERR
{
   //Default constructor
   public:
   LEDERR();
   ~LEDERR();
   void setUpLed(byte ledPins[]);
   void flashLed(byte ledPins[], int duration);
	 void turnOn(byte ledPins[]);
	 void turnOff(byte ledPins[]);
	
};

#endif

