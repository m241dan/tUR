#include "Spec.h"
#include <SD.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define BME_PIN 40
#define SD_PIN 41
#define SAMPLE_RATE 1000
#define BUS_ONE 30
#define BUS_TWO 31
#define BUS_THREE 32


String buf;
int iteration, time_schedule;
Spec SO2_sensor( SPEC_SO2, A0, A1, A2, 48.45 );
Spec O3_sensor( SPEC_O3, A3, A4, A5, 48.45 );
Spec NO2_sensor( SPEC_NO2, A6, A7, A8, 48.45 );
Adafruit_BME280 bme( BME_PIN );
OneWire bus_one( BUS_ONE );
OneWire bus_two( BUS_TWO );
OneWire bus_three( BUS_THREE );
DallasTemperature temp_one( &bus_one );
DallasTemperature temp_two( &bus_two );
DallasTemperature temp_three( &bus_three );

String log_name;
File hasp_log;

void setup()
{
   Serial.begin( 9600 );
   while( !Serial );

   if( !SD.begin( SD_PIN ) )
   {
       Serial.println( "SD did not init." );
       return;
   }

   if( !bme.begin() )
   {
       Serial.println( "BME did not init." );
       return;
   }

   log_name = getNextFileName();   
   iteration = 0;
   time_schedule = 0;
}

void loop()
{
    /*
     * The scheduler
     */
    if( ( time_schedule + SAMPLE_RATE ) < millis() )
    {
        takeReadings( true, true );
        time_schedule = millis();
    }
}


/*
 * Utility Functions
 */
void takeReadings( bool write_sd, bool write_blu )
{
    float temperature, pressure, humidity;
    
    iteration += 1;
    temperature = 0.0;
    pressure = 0.0;
    humidity = 0.0;

    pressure = bme.readPressure() / 100.0F;
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();

    buf  = String( iteration ) + " ";
    buf += String( pressure ) + " ";
    buf += String( temperature ) + " ";
    buf += String( humidity ) + " ";
    buf += SO2_sensor.generateRawReading( " ", false, false );
    buf += O3_sensor.generateRawReading( " ", false, false );
    buf += NO2_sensor.generateRawReading( " ", false, false ); 

    if( write_blu )
        Serial.println( buf );
    if( write_sd )
    {
        hasp_log = SD.open( log_name.c_str(), FILE_WRITE );
        hasp_log.println( buf.c_str() );
        hasp_log.close();
    }
}

String getNextFileName()
{
    String file_name;
    int file_iteration = 0;
    
   //This loop will basically find us a free file appended 1-2.1 billion, which should be plenty
   file_name = "endurance_master" + String( file_iteration ) + ".txt";
   for( file_iteration = 1; SD.exists( file_name ); file_iteration++ )
      file_name = "run" + String( file_iteration ) + ".txt";

   return file_name;
}
