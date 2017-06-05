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
unsigned long long downlink_schedule;
bool new_slave_reading;
byte which_bank;
String reading_status;
String sd_status;

typedef struct data_set
{
    double so2_total = 0;
    double so2_count = 0;
    double no2_total = 0;
    double no2_count = 0;
    double o3_total = 0;
    double o3_count = 0;
    double temp_total = 0;
    double temp_count = 0;
    double humidity_total = 0;
    double humidity_count = 0;
    double pressure_total = 0;
    double pressure_count = 0;
    double ext_temp_total = 0;
    double ext_temp_count = 0;
    double ext_humidity_total = 0;
    double ext_humidity_count = 0;
    
} DATA_SET;

DATA_SET sample_set;

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
    while( receiveData( Serial1, receive_buffer_slave, slave_index, &slave_reading, nullptr, nullptr ) != TRANS_DATA );
    
    slave_command.command[0] = ACKNOWLEDGE;
    sendCommand( Serial1, slave_command );
    sendData( Serial, (byte *)&slave_reading, sizeof( slave_reading ) );
}

void loop()
{
    checkGround();
    checkSlave();
    if( ( downlink_schedule + 1000 ) < millis() )
    {
        downlinkToHasp();
        downlink_schedule = millis();
    }
    sample();
}

void checkGround( void )
{
    TRANS_TYPE transmission;
    
    if( !Serial.available() )
        return;

    if( ( transmission = receiveData( Serial, receive_buffer_ground, ground_index, nullptr, &master_command, &current_gtp ) ) == TRANS_INCOMPLETE )
        return;

    switch( transmission )
    {
        case TRANS_COMMAND:
            doCommand();
            break;
        case TRANS_GTP:
            parseGTP();
            break;
    }
}

void checkSlave( void )
{
    TRANS_TYPE transmission;

    if( !Serial1.available() )
        return;

    if( ( transmission = receiveData( Serial1, receive_buffer_slave, slave_index, &slave_reading, nullptr, nullptr ) ) != TRANS_DATA )
        return;
    new_slave_reading = true;
}

void downlinkToHasp( void )
{
    SENSOR_READING *to_send;

    //if we are supposed to send bank 2 but don't have a new reading, skip downlinking
    if( which_bank == 2 && !new_slave_reading )
        return;
        
    switch( which_bank )
    {
        case 1:
            prepareMasterReading();
            to_send = &master_reading;
            break;
        case 2:
            to_send = &slave_reading;
            new_slave_reading = false;
            break;    
    }

    //write and send
    writeSD( to_send );
    sendData( Serial, (byte *)to_send, sizeof( SENSOR_READING ) );

    //clean up to get ready for next round
    which_bank = which_bank == 1 ? 2 : 1;
    if( which_bank == 2 )
    {
        slave_command.command[0] = REQUEST_READING;
        sendCommand( Serial1, slave_command );
    }
}

void sample( void )
{   
    sample_set.so2_total += so2.generateReadingPPM();
    sample_set.so2_count++;
    sample_set.no2_total += no2.generateReadingPPM();
    sample_set.no2_count++;
    sample_set.o3_total += o3.generateReadingPPM();
    sample_set.o3_count++;
    sample_set.temp_total += bme.readTemperature();
    sample_set.temp_count++;
    sample_set.humidity_total += bme.readHumidity();
    sample_set.humidity_count++;
    sample_set.pressure_total += bme.readPressure() / 100.0F;
    sample_set.pressure_count++;
    sample_set.ext_temp_total += dongle.readTemperature();
    sample_set.ext_temp_count++;
    sample_set.ext_humidity_total += dongle.readHumidity();
    sample_set.ext_humidity_count++;
}

void prepareMasterReading( void )
{
    double so2_ppm;
    double no2_ppm;
    double o3_ppm;
    double temp;
    double humidity;
    double pressure;
    double ext_temp;
    double ext_humidity;

    so2_ppm = sample_set.so2_total / sample_set.so2_count;
    no2_ppm = sample_set.no2_total / sample_set.no2_count;
    o3_ppm  = sample_set.o3_total / sample_set.o3_count;

    temp = sample_set.temp_total / sample_set.temp_count;
    humidity = sample_set.humidity_total / sample_set.humidity_count;
    pressure = sample_set.pressure_total / sample_set.pressure_count;

    ext_temp = sample_set.ext_temp_total / sample_set.ext_temp_count;
    ext_humidity = sample_set.ext_humidity_total / sample_set.ext_humidity_count;

    memset( &sample_set, 0, sizeof( sample_set ) );

    assignEntry( master_reading.time, C_TIME(), sizeof( master_reading.time ) );
    assignEntry( master_reading.bank, "1", sizeof( master_reading.bank ) );
    assignEntry( master_reading.so2_reading, String( so2_ppm ).c_str(), sizeof( master_reading.so2_reading ) );
    assignEntry( master_reading.no2_reading, String( no2_ppm ).c_str(), sizeof( master_reading.no2_reading ) );
    assignEntry( master_reading.o3_reading, String( o3_ppm ).c_str(), sizeof( master_reading.o3_reading ) );
    if( bme_status != "BIFD" )
    {
       assignEntry( master_reading.temp_reading, String( temp ).c_str(), sizeof( master_reading.temp_reading ) );
       assignEntry( master_reading.pressure_reading, String( pressure ).c_str(), sizeof( master_reading.pressure_reading ) );
       assignEntry( master_reading.humidity_reading, String( humidity ).c_str(), sizeof( master_reading.humidity_reading ) );
    }
    if( am2315_status != "AIFD" )
    {
        assignEntry( master_reading.extt_reading, String( ext_temp ).c_str(), sizeof( master_reading.extt_reading ) );
        assignEntry( master_reading.ext_humidity_reading, String( ext_humidity ).c_str(), sizeof( master_reading.ext_humidity_reading ) );
    }
    assignEntry( master_reading.pump_status, pump_on ? "PUMP ON" : "PUMP OFF", sizeof( master_reading.pump_status ) );
    //place holder for peltier 
    assignEntry( master_reading.sd_status, sd_status.c_str(), sizeof( master_reading.sd_status ) );
    assignEntry( master_reading.reading_status, reading_status.c_str(), sizeof( master_reading.reading_status ) );
}

void writeSD( SENSOR_READING *reading )
{
    File goat_log;
    byte *ptr;

    if( !( goat_log = SD.open( log_name ,FILE_WRITE ) ) )
    {
        sd_status = "WRITEFAIL";
        return;         
    }
    
    ptr = (byte *)&reading->time[0];
    
    for( int x = 0; x < sizeof( SENSOR_READING ); x++ )
    {
        goat_log.write(*ptr++);
    }

    goat_log.close();
    
}

void doCommand( void )
{
    
}

void parseGTP( void )
{
    
}

