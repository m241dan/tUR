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
    if (!bme.begin()) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }
}

void loop()
{
    sensors.requestTemperatures();

    Serial.print( "Therms: " + String(millis()) + " msec, " +
                    String( sensors.getTempCByIndex(0) ) + " C  , " +
                    String( sensors.getTempCByIndex(1) ) + " C\t ");

    Serial.print("BMETemp = ");
    Serial.print(bme.readTemperature());
    Serial.print("C\t");
    Serial.print("BMEPres = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa\t");
    delay(1000);
}
