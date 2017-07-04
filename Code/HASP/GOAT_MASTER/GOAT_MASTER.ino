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
#include "master_types.h"
#include "goat_funcs.h"
#include "states.h"
#include "pump_controller.h"

/*
 * Organized Globals (hopefully, they seem organized \(o.o)/ )
 */
SENSOR_TABLE sensors;
READINGS_TABLE readings;
GROUND_COMMAND ground_command_handle;
STATUS_TABLE statuss;
RECEIVE_BUFFERS buffers;
TIMER_TABLE timers;
DATA_SET sample_set;
HardwareSerial &ground_serial = Serial;
HardwareSerial &slave_serial = Serial1;
HardwareSerial *blu_serial = &Serial2;
pump_controller pump( PUMP_PIN );

state state_machine[MAX_STATE] = { 
                                   receive_ground( ground_command_handle, readings.gtp, ground_serial, buffers.master, buffers.ground_index ),
                                   receive_slave( readings.slave, slave_serial, buffers.slave, buffers.slave_index ),
                                   downlink_ground( readings, sample_set, statuss, timers, ground_serial, blu_serial ),
                                   request_slave_reading( slave_serial ),
                                   command_handler( ground_command_handle ),
                                   timer_handler( timers, readings.gtp, statuss, pump ),
                                   sample( sample_set, sensors )                                                 
};

STATE_ID current_state;

/********************
 *  Local Functions *
 ********************/

void setupMasterSerials()
{
    //Serial to HASP
    ground_serial.begin( 1200 );
    while( !ground_serial );

    //Serial to Slave
    slave_serial.begin( 300 );
    while( !slave_serial );

    /*
     * Don't want to bother figuring out how to handle pointers to serials...
     */
    Serial2.begin( 9600 );
    while( !Serial2 );
}

/*
 * A simple function but...
 * any additional tweaking can be done here
 */
void setupMasterGlobals()
{
    statuss.log_name = getNextFile( LOG_NAME );
    timers.pump_timer = millis() + ( 30 * 1000 ); //Start the pump 30 seconds after start
    timers.downlink_schedule = millis() + ( 20 * 1000 ); //Downlink the first reading 20 seconds from this time
}

void setupMasterSensors( void )
{
    /*
     * Setup the SD
     * Update status message accordingly
     */
    if( !SD.begin( SD_PIN ) )
        statuss.sd_status = "SD INIT F";
    else
        statuss.sd_status = "SD INIT G";
      
    /*
     * Setup the BME
     * Update status message accordingly
     */
    if( !sensors.bme.begin() )
        statuss.bme_status = "BIFD";
    else
        statuss.bme_status = "BIGD";

    /*
     * Setup the AM2315
     * Update status message accordingly
     */
    if( !sensors.dongle.begin() )
        statuss.am2315_status = "AIFD";
    else
        statuss.am2315_status = "AIGD";
}

/*
 * I know, it's a dumb one-liner but it used to be more complicated lol
 */
void initStateMachine()
{
    current_state = &state_machine[SAMPLE];
}

/*
 * This is a very important function. This function handles the state transition
 * if a state does not return a recommended transition. IE if state.run() returns
 * NONE_SPECIFIC. 
 */
void determineTransition()
{
    
}

/******************
 * Main Execution *
 ******************/

void setup()
{
    setupMasterSerials();
    setupMasterGlobals();
    setupMasterSensors();
    initStateMachine();

    /*
     * This delay is in place to give slave time to boot, essentially both Arduinos will
     * be started at the same time. 
     */
    delay( 2000 );
 
    /*
     * This next part could easilly be its own function, but I didn't think it was worth it.
     * It will only ever be done once right here, it's a unique operation. 
     * 
     * TL;DR: prepareInitialDownlink()
     */
     {
         readings.master.header[2] = "\x1\x21";
         readings.master.terminator[2] = "\r\n";
         assignEntry( readings.master.time, C_TIME(), sizeof( readings.master.time ) );
         assignEntry( readings.master.bank, "1", sizeof( readings.master.bank ) );
         assignEntry( readings.master.so2_reading, "0.00", sizeof( readings.master.so2_reading ) );
         assignEntry( readings.master.no2_reading, "0.00", sizeof( readings.master.no2_reading ) );
         assignEntry( readings.master.o3_reading, "0.00", sizeof( readings.master.o3_reading ) );
         assignEntry( readings.master.temp_reading, "0.00", sizeof( readings.master.temp_reading ) );
         assignEntry( readings.master.extt_reading, "0.00", sizeof( readings.master.extt_reading ) );
         assignEntry( readings.master.pressure_reading, "0.00", sizeof( readings.master.pressure_reading ) );
         assignEntry( readings.master.humidity_reading, "0.00", sizeof( readings.master.humidity_reading ) );
         assignEntry( readings.master.ext_humidity_reading, "0.00", sizeof( readings.master.ext_humidity_reading ) );
         assignEntry( readings.master.pump_status, "P: OFF AUTO", sizeof( readings.master.pump_status ) );
         assignEntry( readings.master.bme_status, statuss.bme_status.c_str(), sizeof( readings.master.bme_status ) );
         assignEntry( readings.master.am2315_status, statuss.am2315_status.c_str(), sizeof( readings.master.am2315_status ) );
         assignEntry( readings.master.sd_status, statuss.sd_status.c_str(), sizeof( readings.master.sd_status ) );
         assignEntry( readings.master.reading_status, "FIRST", sizeof( readings.master.reading_status ) );
     }

     /*
      * Downlink the prepared reading to two possible places:
      * 1.) Downlink to HASP
      * 2.) Downlink to Bluetooth (if the BT hack is still in place )
      */
    sendData( ground_serial, (byte *)&readings.master, sizeof( readings.master ) );
    sendData( *blu_serial, (byte *)&readings.master, sizeof( readings.master ) );

    /*
     * Once the data has been downlinked...
     * Wait for Slave to send its initial prepared readings (like Master does)
     */
    while( !slave_serial.available() );
    while( receiveData( slave_serial, buffers.slave, buffers.slave_index ) != TRANS_DATA );
    
    /*
     * I don't want to check the boolean here, if we got anything from slave it must have initialized
     * and later we will simply throw out corrupt readings and request new ones
     */
    bufferToReading( buffers.slave, readings.slave );
  
    sendCommand( slave_serial, ACKNOWLEDGE );
    sendData( ground_serial, (byte *)&slave_reading, sizeof( slave_reading ) );
    sendData( *blu_serial, (byte *)&slave_reading, sizeof( slave_reading ) );

    /*
     * We start in the Sample State, because it's a good place to start PLUS it should
     */
}

void loop()
{
    current_state = state_machine[current_state].run();
    if( current_state == NONE_SPECIFIC )
        current_state = determineTransition();
}
