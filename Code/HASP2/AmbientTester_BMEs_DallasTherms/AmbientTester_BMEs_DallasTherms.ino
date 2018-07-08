#include <DallasTemperature.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include "ram_registers.h"

#define THERMO_BANK_10     10
#define THERMO_BANK_11     11
#define THERMO_BANK_12     12
#define THERMO_BANK_13     13

OneWire                    oneWire_10(THERMO_BANK_10);
OneWire                    oneWire_11(THERMO_BANK_11);
OneWire                    oneWire_12(THERMO_BANK_12);
OneWire                    oneWire_13(THERMO_BANK_13);
DallasTemperature          thermbank_10(&oneWire_10);
DallasTemperature          thermbank_11(&oneWire_11);
DallasTemperature          thermbank_12(&oneWire_12);
DallasTemperature          thermbank_13(&oneWire_13);
Adafruit_BME280            bme00(BME_CS_00, BME_MOSI, BME_MISO, BME_SCK); // software SPI
Adafruit_BME280            bme01(BME_CS_01, BME_MOSI, BME_MISO, BME_SCK); // software SPI

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
  /*
  // set the Dallas thermometer resolution to 9 bit - Valid values are 9, 10, or 11 bit.
  thermbank_10.setResolution(thermometer_08, 9);
  thermbank_10.setResolution(thermometer_10, 9);
  */
}

void loop()
{
    thermbank_10.requestTemperaturesByAddress(
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
