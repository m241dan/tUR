#include <DallasTemperature.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"

#define THERMO 9
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11 
#define BME_CS 10

OneWire oneWire( THERMO );
DallasTemperature sensors( &oneWire );
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

void setup()
{
    Serial.begin( 9600 );
    while( !Serial );
    if (!bme.begin()) 
    {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
    }
}

void loop()
{
    sensors.requestTemperatures();
    int secs = millis() / 1000; //Oh my God I'm such a kludge. -JA
    Serial.print( String(secs) + " s, \t" +
                    String( sensors.getTempCByIndex(0) ) + " C\t " +
                    String( sensors.getTempCByIndex(1) ) + " C\t " +
                    String( sensors.getTempCByIndex(2) ) + " C\t " +
                    String( sensors.getTempCByIndex(3) ) + " C\t ");

    Serial.print("BMETemp = ");
    Serial.print(bme.readTemperature());
    Serial.print("C\t");
    Serial.print("BMEPres = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa\t");
    delay(1000);
}
