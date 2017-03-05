#include "Spec.h"

Spec *sulfur_sensor;

void setup()
{
   //Assume the sensor gets its power from a constant source
   sulfur_sensor = new Spec( SPEC_SO2, 0, 1, 2 )
}

void loop()
{
   // put your main code here, to run repeatedly:
   delay(2000);
   Serial.print( sulfur_sensor->generateReading() );
}
