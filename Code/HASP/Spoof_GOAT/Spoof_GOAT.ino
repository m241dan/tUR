/*
 * Spoof the GOAT
 */

#include "hasp_types.h"

SENSOR_READING reading;
unsigned long long time_schedule;

void setup()
{
    Serial.begin( 1200 );
    while( !Serial );

    time_schedule = 0;
}

void loop()
{
    if( ( time_schedule + 2000 ) < millis() )
    {
        assignEntry( reading.time, String( millis() ).c_str(), sizeof( reading.time ) );
        Serial.print( String( millis() ) );
//        reading.bank = "1 ";
  //      reading.so2_reading = "20.00";
    //    reading.o3_reading = "15.00";
      //  reading.no2_reading = "35.00";
       // reading.temp_reading = "25.25";
        //reading.pressure_reading = "1550.50";
       // reading.pump_status = "ON";
       // reading.peltier_status = "OFF";
       // reading.sd_status = "GOOD";
       // reading.reading_status = " ACTIVE";
        Serial.write( (byte [])&reading, 133 ); 
    }

}

void assignEntry( char *dst, const char *src, int length )
{
    for( int x = 0; x < length; x++ )
        *dst = *src;
}

