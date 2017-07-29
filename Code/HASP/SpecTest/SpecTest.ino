#include "Spec.h"

Spec NO2_sensor( SPEC_NO2, A0, A1, A2, -50.70, 0.07 );
//Spec O3_sensor( SPEC_O3, A3, A4, A5, -12.89 );
void setup()
{
    Serial.begin( 9600 );
    while( !Serial );
}

void loop()
{
    Serial.println( "NO2 PPM: " + String( NO2_sensor.generateReadingPPM(), 10 ) ); // + " O3 PPM: " + String( O3_sensor.generateReadingPPM(), 10 ) );
    Serial.println( "NO2 RAW: " + NO2_sensor.generateRawReading( ',') ); // + " O3 RAW: " + O3_sensor.generateRawReading(',') );
    delay( 1000);
}

