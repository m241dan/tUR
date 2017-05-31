
#include "slave_globals.h"
#include "goat_slave_funcs.h"
#include "goat_funcs.h"

Spec so2( SPEC_SO2, A0, A1, A2, 43.54 );
Spec no2( SPEC_NO2, A3, A4, A5, 43.53 );
Spec o3( SPEC_O3, A6, A7, A8, 43.56 );
Adafruit_BME280 bme( BME_PIN );
GROUND_COMMAND current_command;
SENSOR_READING reading;
byte receive_buffer[MAX_BUF];
unsigned int buffer_index;

void setup()
{
    setupSlaveSerials();
    setupSlaveGlobals();
    setupSlaveSensors();

    delay( 10000 );
    assignEntry( reading.time, C_TIME(), sizeof( reading.time ) );
    sendData( Serial, (byte *)&reading, sizeof( reading ) );

    while( !Serial.available() ); //block and wait for acknowledgement
    while( 1 )
    {
        while( !receiveData( Serial, receive_buffer, buffer_index, nullptr, &current_command, nullptr ) ); //block until we get the something
        if( current_command.command[0] == ACKNOWLEDGE )
           break;
    }
}   

void loop()
{

}

void checkMaster( void )
{
    
}

void downlinkToMaster( void )
{
    
}

void sample( void )
{
    
}

void prepareReading( void )
{
    
}

void prepareSlaveSample( void )
{
    
}

