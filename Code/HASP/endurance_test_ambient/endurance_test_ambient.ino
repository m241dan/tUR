#include "Spec.h"
#include <SD.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP183.h>

#define BMP_PIN 40
#define SD_PIN 41
#define SAMPLE_RATE 1000

String buf;
int iteration, time_schedule;
Spec SO2_sensor( SPEC_SO2, A0, A1, A2, 48.45 );
Spec O3_sensor( SPEC_O3, A3, A4, A5, 48.45 );
Spec NO2_sensor( SPEC_NO2, A6, A7, A8, 48.45 );
Adafruit_BMP183 bmp( BMP_PIN );

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

   if( !bmp.begin() )
   {
       Serial.println( "BMP did not init." );
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
    float temperature, pressure;
    
    iteration += 1;
    temperature = 0.0;
    pressure = 0.0;

    pressure = bmp.getPressure() / 100.0F;
    temperature = bmp.getTemperature();

    buf  = String( iteration ) + " ";
    buf += String( pressure ) + " ";
    buf += String( temperature ) + " ";
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
