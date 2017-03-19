#include "Arduino.h"

std::array<String, 5> hasp_commands;
std::array<String, 5> slave_messages;


void setup()
{
   Serial.begin( 1200 ); //connection to HASP
   while( !Serial );

   Serial1.begin( 115200 ); //connect to the Slave
   while( !Serial1 );

}

void loop()
{

}

void serialEvent()
{
   while( Serial.available() )
   {
   }
}

void serialEvent1()
{

}
