#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include "Arduino.h"
#include "HardwareSerial.h"
#include "hasp_types.h"
#include "goat_funcs.h"
#include "master_types.h"


void setupMasterSerials( void )
{    //Serial to HASP
    Serial.begin( 1200 );
    while( !Serial );

    //Serial to Slave
    Serial1.begin( 300 );
    while( !Serial1 );
}

void setupMasterGlobals( void )
{
    log_name = getNextFile( LOG_NAME );


    //setup the sensor reading message as its how we communicate
    assignEntry( master_reading.time, C_TIME(), sizeof( master_reading.time ) );
    assignEntry( master_reading.bank, "1", sizeof( master_reading.bank ) );
    assignEntry( master_reading.pump_status, "PUMP OFF", sizeof( master_reading.pump_status ) );
    assignEntry( master_reading.reading_status, "FIRST", sizeof( master_reading.reading_status ) );
}

void setupMasterSensors( void )
{
    //Setup the SD
    if( !SD.begin( SD_PIN ) )
        sd_status = "SD INIT F";
    else
    {
        sd_status = "SD INIT G";
        log_name = getNextFile( "GOAT" );
    }
    assignEntry( master_reading.sd_status, sd_status.c_str(), sizeof( master_reading.sd_status ) );

    //Setup the BME
    if( !bme.begin() )
        bme_status = "BIFD";
    else
        bme_status = "BIGD";
    assignEntry( master_reading.temp_reading, bme_status.c_str(), sizeof( master_reading.temp_reading ) );


    //Setup the AM2315
    if( !dongle.begin() )
        am2315_status = "AIFD";
    else
        am2315_status = "AIGD";
    assignEntry( master_reading.extt_reading, am2315_status.c_str(), sizeof( master_reading.extt_reading ) );

}
