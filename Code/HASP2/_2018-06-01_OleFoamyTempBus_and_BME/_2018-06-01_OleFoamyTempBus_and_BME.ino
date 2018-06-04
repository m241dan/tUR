#include <DallasTemperature.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"

#define THERMO_CH1 9 //WHT
#define THERMO_CH2 8 //BLU
#define THERMO_CH3 7 //WHT
#define THERMO_CH4 6 //BLU
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11 
#define BME_CS 10

OneWire oneWire_ch1( THERMO_CH1 );
OneWire oneWire_ch2( THERMO_CH2 );
OneWire oneWire_ch3( THERMO_CH3 );
OneWire oneWire_ch4( THERMO_CH4 );
DallasTemperature sensors_ch1( &oneWire_ch1 );
DallasTemperature sensors_ch2( &oneWire_ch2 );
DallasTemperature sensors_ch3( &oneWire_ch3 );
DallasTemperature sensors_ch4( &oneWire_ch4 );
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
    sensors_ch1.requestTemperatures();
    sensors_ch2.requestTemperatures();
    int secs = millis() / 1000; //Oh my God I'm such a kludge. -JA
    Serial.println("*------------------------------------------*");
    Serial.print( String(secs) + " s, \tBANK 1: " + 
                    String( sensors_ch1.getTempCByIndex(0) ) + " " +
                    String( sensors_ch1.getTempCByIndex(1) ) + " " +
                    String( sensors_ch1.getTempCByIndex(2) ) + " " +
                    String( sensors_ch1.getTempCByIndex(3) ) + "\tBANK 2: " + 
                    String( sensors_ch2.getTempCByIndex(0) ) + " " +
                    String( sensors_ch2.getTempCByIndex(1) ) + " " +
                    String( sensors_ch2.getTempCByIndex(2) ) + " " +
                    String( sensors_ch2.getTempCByIndex(3) ) + "\n\tBANK 3: " +
                    String( sensors_ch3.getTempCByIndex(0) ) + " " +
                    String( sensors_ch3.getTempCByIndex(1) ) + " " +
                    String( sensors_ch3.getTempCByIndex(2) ) + " " +
                    String( sensors_ch3.getTempCByIndex(3) ) + "\tBANK 4: " +
                    String( sensors_ch4.getTempCByIndex(0) ) + " " +
                    String( sensors_ch4.getTempCByIndex(1) ) + " " +
                    String( sensors_ch4.getTempCByIndex(2) ) + " " +
                    String( sensors_ch4.getTempCByIndex(3) ) + "\n");
    Serial.print("\tBMETemp = ");
    Serial.print(bme.readTemperature());
    Serial.print("C\t");
    Serial.print("BMEPres = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa\t");
    Serial.println("*------------------------------------------*\n");
    delay(1000);
}
