/*
 * HASP Gondola: Simulator
 * Author: Daniel R. Koris, Erick Rameriz
 * Goal of the Program: The goal of this program is to simulate the HASP Gondola.
 * Required Components: Two Arduinos wired through Rx and Tx, 1200 Baud rate, Serial Uplink and Downlink. 
 */

#include "hasp_types.h"
#include "goat_funcs.h"

//Telemetry Rate
#define TELE_RATE 60000

//GPS Telemetry
SAMPLE_GTP spoof_gps;


//Global Variables
unsigned long long time_schedule;
unsigned long long time_command;

byte possible_commands[10] = { ARD_RESET, DOWNLINK_OFF, DOWNLINK_ON, STOP_SENSORS, START_SENSORS, PUMP_ON, PUMP_OFF, DISABLE_SD, ENABLE_SD, REINIT_SD };

void setup()
{
    Serial.begin( 1200 );
    while( !Serial );
    
    time_schedule = 0;
}

void loop()
{
    if( ( time_schedule + TELE_RATE ) < millis() )
    {
        //Send telemetry data every minute
        sendData( Serial, (byte *)&spoof_gps, sizeof( spoof_gps ) );
        time_schedule = millis();
    }

/*
    if( Serial.available() )
    {
        GROUND_COMMAND spoof_command;
        int received_command = Serial.parseInt();
        Serial.println( "Sending Command: " + String( received_command ) );
        //Send command
        spoof_command.command[0] = possible_commands[received_command];
        sendData( Serial1, (byte *)&spoof_command, sizeof( GROUND_COMMAND ) );
    } */
}
