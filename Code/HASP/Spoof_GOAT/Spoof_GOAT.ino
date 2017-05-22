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
        //assignment -- this is how we have to build data into our readings with the proper fill
        assignEntry( reading.time, String( millis() ).c_str(), sizeof( reading.time ) );
        assignEntry( reading.bank, "12", sizeof( reading.bank ) );
        assignEntry( reading.so2_reading, "20.00", sizeof( reading.so2_reading ) );
        assignEntry( reading.o3_reading, "15.00", sizeof( reading.o3_reading ) );
        assignEntry( reading.no2_reading, "35.00", sizeof( reading.no2_reading ) );
        assignEntry( reading.temp_reading, "25.25", sizeof( reading.temp_reading ) );
        assignEntry( reading.pressure_reading, "1550.50", sizeof( reading.pressure_reading ) );
        assignEntry( reading.pump_status, "ON", sizeof( reading.pump_status ) );
        assignEntry( reading.peltier_status, "OFF", sizeof( reading.peltier_status ) );
        assignEntry( reading.sd_status, "GOOD", sizeof( reading.sd_status ) );
        assignEntry( reading.reading_status, " ACTIVE", sizeof( reading.reading_status ) );
        //downlinking -- this is the logic to send data down to gondola (simple, works, beautiful)
        byte *ptr;
        ptr = (byte *)&reading;
        for( int x = 0; x < 133; x++ )
        {
            Serial.write( *ptr );
            ptr++;
        }
        Serial.println();
    }

}

//this function is specifically designed to fill in our readings entries with proper space padding and terminating each with a ','
void assignEntry( char *dst, const char *src, int length )
{
    for( int x = 0; x < length; x++ )
    {
        //value at src and put it into dst and then increment
        *dst = *src;
        dst++;
        src++;
        //if we are at the end of src (meaning always send '\0' terminated c strings as src) fill with spaces
        if( *src == '\0' )
        {
            for( int y = 0; y < ( (length-1) - x ); y++ )
            {
               *dst = ' ';
               dst++;
            }
            break;
        }
    }
    //terminate with ','
    *(dst-1) = ',';
}

