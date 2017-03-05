#include "Spec.h"

Spec *sulfur_sensor;

void setup()
{
   Serial.begin( 115200 );
   while( !Serial );

   //Assume the sensor gets its power from a constant source
   sulfur_sensor = new Spec( SPEC_SO2, 0, 1, 2 );
}

void loop()
{
   delay(2000);
   if( Serial.available() )
      Serial.println( sulfur_sensor->generateReading() );
}
