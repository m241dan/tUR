/*
 * HASP GOAT: Master Arduino
 * Author: Daniel R. Koris
 * Goal of Program: Issuing commands and receiving readings from the Slave Arduino,
 *                  receiving commands and telemtry from HASP Gondola,
 *                  taking readings from one half of the sensor rig,
 *                  control the Sparkfun pump,
 *                  and thermal controlling.
 * Required Components: Arduino Mega, Adafruit MicroSD Card Breakout Board, Adafruit BME280,
 *                      Spec SO2, Spec O3, Spec NO2, Adafruit Oscillator, Resistor Heaters,
 *                      Thermoelectric Modules
 */

#include "goat_master_funcs.h"
#include "master_globals.h"
#include "goat_funcs.h"

Spec so2( SPEC_SO2, A0, A1, A2, 43.54 );
Spec no2( SPEC_NO2, A3, A4, A5, 43.54 );
Spec o3( SPEC_O3, A6, A7, A8, 43.54 );
Adafruit_BME280 bme( BME_PIN );
Adafruit_AM2315 dongle;
String log_name;
GROUND_COMMAND master_command;
GROUND_COMMAND slave_command;
GTP_DATA current_gtp;
SENSOR_READING master_reading;
SENSOR_READING slave_reading;
bool pump_on;
bool take_readings;
byte receive_buffer_ground[MAX_BUF];
unsigned int ground_index;
byte receive_buffer_slave[MAX_BUF];
unsigned int slave_index;

void setup()
{
    setupMasterSerials();
    setupMasterGlobals();
    setupMasterSensors();

    delay( 2000 );
    assignEntry( master_reading.time, C_TIME(), sizeof( master_reading.time ) );
    sendData( Serial, (byte *)&master_reading, sizeof( master_reading ) );

    //block until we get a response from slave
    while( !Serial1.available() );
    while( !receiveData( Serial1, receive_buffer_slave, slave_index, &slave_reading, nullptr, nullptr ) )

    slave_command.command[0] = ACKNOWLEDGE;
    sendCommand( Serial1, slave_command );
    sendData( Serial, (byte *)&slave_reading, sizeof( slave_reading ) );
    

}

void loop()
{

}

