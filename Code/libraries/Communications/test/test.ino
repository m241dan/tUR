#include "Arduino.h"
#include "Hasp_Types.h"

std::array<String, 5> hasp_commands;
std::array<String, 5> slave_messages;

byte hasp_rx_buffer[256];

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
   
}

void serialEvent1()
{

}
