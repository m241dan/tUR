#include <DallasTemperature.h>

#define THERMO 11

OneWire oneWire( THERMO );
DallasTemperature sensors( &oneWire );

void setup()
{
    Serial.begin( 9600 );
    while( !Serial );
}

void loop()
{
    sensors.requestTemperatures();

    Serial.println( String( millis() ) + "ms, \t" +
                    String( sensors.getTempCByIndex(0) ) + " C, \t" +
                    String( sensors.getTempCByIndex(1) ) );
}
