#include <DallasTemperature.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"

#define THERMO_01 10 //Bank01
#define THERMO_02 11 //Bank02
#define THERMO_03 12 //Bank03
#define THERMO_04 13 //Bank04
#define BME_SCK 24
#define BME_MOSI 23
#define BME_MISO 22 
#define BME_CS_01 1
#define BME_CS_00 0

OneWire oneWire_01( THERMO_01 );
OneWire oneWire_02( THERMO_02 );
OneWire oneWire_03( THERMO_03 );
OneWire oneWire_04( THERMO_04 );
DallasTemperature sensors_01( &oneWire_01 );
DallasTemperature sensors_02( &oneWire_02 );
DallasTemperature sensors_03( &oneWire_03 );
DallasTemperature sensors_04( &oneWire_04 );
Adafruit_BME280 bme00(BME_CS_00, BME_MOSI, BME_MISO, BME_SCK); // software SPI
Adafruit_BME280 bme01(BME_CS_01, BME_MOSI, BME_MISO, BME_SCK); // software SPI

void setup()
{
    Serial.begin( 9600 );
    while( !Serial );
    if ( (!bme00.begin()) || (!bme01.begin()))
    {
      Serial.println("One of your BMEs is dedAF; check wiring!");
      if ( (!bme00.begin()) && (!bme01.begin()))
      {
        Serial.println("Could not find any valid BME280 sensor, check wiring!");
      }
    }
}

void loop()
{
    sensors_01.requestTemperatures();
    sensors_02.requestTemperatures();
    sensors_03.requestTemperatures();
    sensors_04.requestTemperatures();
    int secs = millis() / 1000; 
    Serial.println( String(secs) + " s,");
    Serial.println("Bank01\t" +
                    String( sensors_01.getTempCByIndex(0) ) + " C\t " +
                    String( sensors_01.getTempCByIndex(1) ) + " C\t " +
                    String( sensors_01.getTempCByIndex(2) ) + " C\t " +
                    String( sensors_01.getTempCByIndex(3) ) + " C\t ");

    Serial.println( "Bank02\t" +
                    String( sensors_02.getTempCByIndex(0) ) + " C\t " +
                    String( sensors_02.getTempCByIndex(1) ) + " C\t " +
                    String( sensors_02.getTempCByIndex(2) ) + " C\t " +
                    String( sensors_02.getTempCByIndex(3) ) + " C\t ");
    Serial.println( "Bank03\t" +
                    String( sensors_03.getTempCByIndex(0) ) + " C\t " +
                    String( sensors_03.getTempCByIndex(1) ) + " C\t " +
                    String( sensors_03.getTempCByIndex(2) ) + " C\t " +
                    String( sensors_03.getTempCByIndex(3) ) + " C\t ");
    Serial.println( "Bank04\t" +
                    String( sensors_04.getTempCByIndex(0) ) + " C\t " +
                    String( sensors_04.getTempCByIndex(1) ) + " C\t " +
                    String( sensors_04.getTempCByIndex(2) ) + " C\t " +
                    String( sensors_04.getTempCByIndex(3) ) + " C\t ");
/*
    Serial.print("BME00 = ");
    Serial.print(bme00.readTemperature());
    Serial.print("C,\t");
    Serial.print(bme00.readPressure() / 100.0F);
    Serial.print(" hPa\t");
    Serial.print(bme00.readHumidity());
    Serial.print("%,\t");

    Serial.print("BME01 = ");
    Serial.print(bme01.readTemperature());
    Serial.print("C,\t");
    Serial.print(bme01.readPressure() / 100.0F);
    Serial.print(" hPa\t");
    Serial.print(bme00.readHumidity());
    Serial.println("%,\t");
    */
    delay(2500);
}
