#include <SD.h>
#include "Arduino.h"
#include "HardwareSerial.h"
#include "hasp_types.h"
#include "goat_funcs.h"

void sendData( byte *data, int length )
{
    for( int x = 0; x < length; x++ )
    {
        Serial.write( *data );
        data++;
    }
}

void sendData1( byte *data, int length )
{
    for( int x = 0; x < length; x++ )
    {
       Serial1.write( *data );
       data++;
    }
}

//this function is specifically designed to fill in our readings entries with proper space padding and terminating each with a ','
void assignEntry( char *dst, const char *src, int length, bool from_uplink = false )
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
    if( !from_uplink )
        *(dst-1) = ',';
}


String getNextFile( String name )
{
   String file_name;
   int file_iteration = 1;

   //This loop will basically find us a free file appended 1-2.1 billion, which should be plenty
   file_name = name + String( file_iteration ) + ".txt";
   for( ; SD.exists( file_name ); file_iteration++ )
      file_name = name + String( file_iteration ) + ".txt";

   return file_name;
}

void setupMasterSerials( void )
{    //Serial to HASP
    Serial.begin( 1200 );
    while( !Serial );

    //Serial to Slave
    Serial1.begin( 9600 );
    while( !Serial1 );
}

void setupMasterSensors( void )
{
    //Setup the SD
    if( !SD.begin( SD_PIN ) )
       assignEntry( reading.sd_status, "SD FAILED", sizeof( reading.sd_status ) );
    else
       assignEntry( reading.sd_status,M "SD GOOD", sizeof( reading.sd_stats ) );

    //Setup the BME
    if( !bme.begin() )
       assignEntry( reading.temp_reading, "BFAIL", sizeof( reading.temp_reading ) );
    else
       assignEntry( reading.temp_reading, "BGOOD", sizeof( reading.temp_reading )  );


    //Setup the AM2315
    if( !am2315.begin() )
       assignEntry( reading.extt_reading, "AFAIL", sizeof( reading.extt_reading ) );
    else
       assignEntry( reading.extt_reading, "AGOOD", sizeof( reading.extt_reading ) );

}

void setupMasterGlobals( void )
{
    log_name = getNextFile( "GOAT" );
    pump_on = false;
    take_readings = true;

    //clear these out appropriately
    memset( &current_gtp.data[0], 0, sizeof( current_gtp.data ) );
    memset( &master_reading, ' ', sizeof( master_reading ) - 2 );
    memset( &slave_reading, ' ', sizeof( slave_reading ) - 2 );
    memset( &receive_buffer_ground[0], 0, MAX_BUF );
    memset( &receive_buffer_slave[0], 0, MAX_BUF );

    //setup the sensor reading message as its how we communicate 
    assignEntry( master_reading.time, C_TIME(), sizeof( master_reading.time ) );
    assignEntry( master_reading.bank, "1", sizeof( master_reading.bank ) );
    assignEntry( master_reading.pump_status, "PUMP OFF", sizeof( master_reading.pump_status ) );
    assignEntry( master_reading.reading_status, "FIRST", sizeof( master_reading.reading_status ) );
}

