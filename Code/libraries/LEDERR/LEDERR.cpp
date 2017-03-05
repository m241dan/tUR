#include "LEDERR.h"

//<<constructor>> 
LEDERR::LEDERR()
{
}

//<<destructor>>
LEDERR::~LEDERR()
{
}

//Setup the Led pins
void LEDERR::setUpLed(byte ledPins[])
{
   for (int i = 0; i <= sizeof(ledPins); i++)
      pinMode(ledPins[i], OUTPUT);
}

//Flash the Led/Leds
void LEDERR::flashLed(byte ledPins[], int duration)
{
   for (int i = 0; i <= sizeof(ledPins); i++)
   {
      digitalWrite(ledPins[i], HIGH);
      delay(duration/2);
      digitalWrite(ledPins[i], LOW);
      delay(duration/2);
   }
}

void LEDERR::turnOn(byte ledPins[])
{
   for(int i = 0; i <= sizeof(ledPins); i++)
      digitalWrite(ledPins[i], HIGH);
}

void LEDERR::turnOff(byte ledPins[])
{
   for(int i = 0; i <= sizeof(ledPins); i++)
      digitalWrite(ledPins[i], LOW);
}


