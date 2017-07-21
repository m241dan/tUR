#include "Spec.h"

Spec SO2_sensor( SPEC_NO2, A0, A1, A2, -52.50 );

void setup()
{
    Serial.begin( 9600 );
    while( !Serial );
}

void loop()
{
    Serial.println( String( SO2_sensor.generateReadingPPM() ) );
    Serial.println( SO2_sensor.generateRawReading( ',') );
    delay( 1000);
}

