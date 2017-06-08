#include "slave_globals.h"
#include "Arduino.h"
#include "HardwareSerial.h"
#include "hasp_types.h"
#include "goat_funcs.h"
#include "goat_slave_funcs.h"

void setupSlaveSerials( void )
{
   //Serial to Master
   Serial.begin( 300 );
   while( !Serial );
}

void setupSlaveGlobals( void )
{
    memset( &slave_reading.time[0], ' ', sizeof( slave_reading ) - 4 );
    memset( &receive_buffer[0], 0, MAX_BUF );
    buffer_index = 0;

    assignEntry( slave_reading.time, C_TIME(), sizeof( slave_reading.time ) );
    assignEntry( slave_reading.bank, "2", sizeof( slave_reading.bank ) );
    assignEntry( slave_reading.extt_reading, "DNA", sizeof( slave_reading.extt_reading ) );
    assignEntry( slave_reading.ext_humidity_reading, "DNA", sizeof( slave_reading.ext_humidity_reading ) );
    assignEntry( slave_reading.pump_status, "DNA", sizeof( slave_reading.pump_status ) );
    assignEntry( slave_reading.sd_status, "DNA", sizeof( slave_reading.sd_status ) );
    assignEntry( slave_reading.reading_status, "FIRST", sizeof( slave_reading.reading_status ) );
}

void setupSlaveSensors( void )
{
    if( !bme.begin() )
       bme_status = "BIFD";
    else
       bme_status = "BIGD";
    assignEntry( slave_reading.temp_reading, bme_status.c_str(), sizeof( slave_reading.temp_reading ) );
}
