#include "Spec.h"

Spec SO2_sensor( SPEC_SO2, A8, A4, A5, 43.45 );

void setup()
{
    Serial.begin( 9600 );
    while( !Serial );
}

void loop()
{
    Serial.print( SO2_sensor.generateRawReading() );
    delay( 1000);
}

