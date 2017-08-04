#include <Adafruit_ADS1015.h>
#include "Spec.h"

Adafruit_ADS1115 ads;

//Spec SO2_sensor( SPEC_SO2, A0, A1, A2, 36.89, 0 );
//Spec NO2_sensor( SPEC_NO2, -52.50, -1.37468 ); //0.07 );
Spec O3_sensor( SPEC_O3, A0, A1, A2, -12.89 );
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
 
    //Serial.println( "NO2 RAW: " + NO2_sensor.generateRawReading( ',') ); // + " O3 RAW: " + O3_sensor.generateRawReading(',') );
    //double differential = ( ads.readADC_Differential_0_1() * 0.065 ) / 1000.00;
    //Serial.println( "Differential: " + String( differential, 10 ) );
    //Serial.println( "NO2 PPM: " + String( NO2_sensor.generateReadingPPM( differential ), 10 ) ); // + " O3 PPM: " + String( O3_sensor.generateReadingPPM(), 10 ) );    
 //   Serial.println( "Analog 0: " + String( ads.readADC_SingleEnded(0) * 0.065 ) );
  //  Serial.println( "Analog 1: " + String( ads.readADC_SingleEnded(1) * 0.065) );
//    Serial.println( NO2_sensor.generateRawReading( ',' ) );
 
    delay( 500 );
    Serial.println( "O3 RAW: " + O3_sensor.generateRawReading( ',' ) );
    double differential = ( ads.readADC_Differential_0_1() * 0.065 ) / 1000.00;
    Serial.println( "Differential: " + String( differential, 10 ) );
}

