/*
 * HASP Gondola: Simulator
 * Author: Daniel R. Koris, Erick Rameriz
 * Goal of the Program: The goal of this program is to simulate the HASP Gondola.
 * Required Components: Two Arduinos wired through Rx and Tx, 1200 Baud rate, Serial Uplink and Downlink. 
 */

#include <SD.h>
#include <SPI.h>
#include "hasp_types.h"

//Telemetry Rate
#define TELE_RATE 60000
#define SD_PIN 10

//GPS Telemetry
GTP_DATA spoof_gps;

//Global Variables
unsigned long long time_schedule;
String log_name;

void setup()
{
    //Begin Serial Communication
    Serial.begin( 1200 );
    while( !Serial );

    if( !SD.begin( 10 ) )
        Serial.println( "SD did not init." );

    time_schedule = 0;
    log_name = getNextFile( "simData" );    
}

void loop()
{
    if( ( time_schedule + TELE_RATE ) < millis() )
    {
        //Send telemetry data every minute
        sendData( (byte *)&spoof_gps, sizeof( spoof_gps ) );
        time_schedule = millis();
    }
    else
    {
        //Check for received data
        checkReceiveData();    
    }
}

void checkReceiveData()
{
    File sim_log;
    if( Serial.available() )
    {
        if( !( sim_log = SD.open( log_name, FILE_WRITE ) ) )
        {
            Serial.println( "File cannot be opened." );
            return;
        }
        while( Serial.available() )
            sim_log.write( Serial.read() );
        sim_log.close();
    }
}
