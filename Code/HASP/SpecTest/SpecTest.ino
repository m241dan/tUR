#include <Adafruit_ADS1015.h>
#include "Spec.h"

Adafruit_ADS1115 ads;

//Spec SO2_sensor( SPEC_SO2, A0, A1, A2, 36.89, 0 );
Spec NO2_sensor( SPEC_NO2, A0, A1, A2, -50.70, 0 ); //0.07 );
//Spec O3_sensor( SPEC_O3, A3, A4, A5, -12.89 );
void setup()
{
    Serial.begin( 9600 );
    while( !Serial );

    ads.setGain( GAIN_TWO );
    ads.begin();
}

void loop()
{
//    Serial.println( "SO2 RAW: " + SO2_sensor.generateRawReading( ',' ) );
 //   Serial.println( "NO2 PPM: " + String( NO2_sensor.generateReadingPPM(), 10 ) ); // + " O3 PPM: " + String( O3_sensor.generateReadingPPM(), 10 ) );
    Serial.println( "NO2 RAW: " + NO2_sensor.generateRawReading( ',') ); // + " O3 RAW: " + O3_sensor.generateRawReading(',') );
    Serial.println( "Differential: " + String( ads.readADC_Differential_0_1() ) );
    Serial.println( "Analog 0: " + String( ads.readADC_SingleEnded(0) ) );
    Serial.println( "Analog 1: " + String( ads.readADC_SingleEnded(1) ) );
//    Serial.println( NO2_sensor.generateRawReading( ',' ) );
    delay( 500 );
}

