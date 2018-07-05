//#include <DallasTemperature.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"

#define THERMO 9
#define BME_SCK 24
#define BME_MOSI 23
#define BME_MISO 22 
#define BME_CS_01 1
#define BME_CS_00 0

//OneWire oneWire( THERMO );
//DallasTemperature sensors( &oneWire );
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
/*    sensors.requestTemperatures();
    int secs = millis() / 1000; //Oh my God I'm such a kludge. -JA
    Serial.print( String(secs) + " s, \t" +
                    String( sensors.getTempCByIndex(0) ) + " C\t " +
                    String( sensors.getTempCByIndex(1) ) + " C\t " +
                    String( sensors.getTempCByIndex(2) ) + " C\t " +
                    String( sensors.getTempCByIndex(3) ) + " C\t ");
*/
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
    
    delay(1000);
}
