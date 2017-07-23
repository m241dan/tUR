#include "Spec.h"

Spec SO2_sensor( SPEC_NO2, A4, A1, A2, -52.50 );

void setup()
{
    Serial.begin( 9600 );
    while( !Serial );
}

void loop()
{
    Serial.println( "PPM: " + String( SO2_sensor.generateReadingPPM(), 10 ) );
 //   Serial.println( "RAW: " + SO2_sensor.generateRawReading( ',') );
    delay( 1000);
}

