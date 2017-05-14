/*
 * HASP ST: Simple Thermometer Test Rig
 * Author: Daniel R. Koris
 * Goal of Program: To get two temperature readings from two thermometers
 * Required Components: Arduino (any), Adafruit DS18B20 Digital Temperature Sensor (374)
 */

/*
 * libraries
 */

#include <OneWire.h>
#include <DallasTemperature.h>

/*
 * globals/configs
 */

//which digital pin on the Arduino?
OneWire oneWire(6);
DallasTemperature sensors( &oneWire );

void setup()
{

    Serial.begin( 9600 );
    while( !Serial );

    sensors.begin();
    Serial.println( "Initialization complete." );
}

void loop()
{
    delay( 1000 );
    Serial.print( "Requesting Reading..." );
    sensors.requestTemperatures();
    Serial.println( "DONE" );
    Serial.println( "Thermo #1: " + String( sensors.getTempCByIndex(0) ) + " Thermo #2: " + String( sensors.getTempCByIndex(1) ) );
}

