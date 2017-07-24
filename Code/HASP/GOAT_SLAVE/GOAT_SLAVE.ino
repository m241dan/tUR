
#include "slave_globals.h"
#include "goat_funcs.h"

Spec so2( SPEC_SO2, A0, A1, A2, 43.45 );
Spec no2( SPEC_NO2, A3, A4, A5, 51.63 );
Spec o3( SPEC_O3, A6, A7, A8, 10.87 );
Adafruit_BME280 bme( BME_PIN );
byte receive_buffer[MAX_BUF];
unsigned int buffer_index;
String reading_status = "FIRST";
String bme_status;

//Data set struct that holds our data
struct slave_data_set
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
    
} sample_set;

void setupSlaveSerials( void )
{
   //Serial to Master
   Serial.begin( 300 );
   while( !Serial );
}

void setupSlaveGlobals( void )
{
    memset( &receive_buffer[0], 0, MAX_BUF );
    buffer_index = 0;
}

void setupSlaveSensors( void )
{
    if( !bme.begin() )
       bme_status = "BIFD";
    else
       bme_status = "BIGD";
}

void setup()
{
    SENSOR_READING slave_reading;
    GROUND_COMMAND current_command;
    //Setup and initialize sensors
    setupSlaveSerials();
    setupSlaveGlobals();
    setupSlaveSensors();

    delay( 10000 );

    slave_reading.header[0] = '\x01';
    slave_reading.header[1] = '\x21';
    slave_reading.terminator[0] = '\r';
    slave_reading.terminator[1] = '\n';
    assignEntry( slave_reading.time, C_TIME(), sizeof( slave_reading.time ) );
    assignEntry( slave_reading.bank, "2", sizeof( slave_reading.bank ) );
    assignEntry( slave_reading.so2_reading, "0.00", sizeof( slave_reading.so2_reading ) );
    assignEntry( slave_reading.no2_reading, "0.00", sizeof( slave_reading.no2_reading ) );
    assignEntry( slave_reading.o3_reading, "0.00", sizeof( slave_reading.o3_reading ) );
    assignEntry( slave_reading.temp_reading, "0.00", sizeof( slave_reading.temp_reading ) );
    assignEntry( slave_reading.extt_reading, "SLAVE", sizeof( slave_reading.extt_reading ) );
    assignEntry( slave_reading.pressure_reading, "0.00", sizeof( slave_reading.pressure_reading ) );
    assignEntry( slave_reading.humidity_reading, "0%", sizeof( slave_reading.humidity_reading ) );
    assignEntry( slave_reading.ext_humidity_reading, "SLAVE", sizeof( slave_reading.ext_humidity_reading ) );
    assignEntry( slave_reading.pump_status, "SLAVE", sizeof( slave_reading.pump_status ) );
    assignEntry( slave_reading.bme_status, bme_status.c_str(), sizeof( slave_reading.bme_status ) );
    assignEntry( slave_reading.am2315_status, "SLAVE", sizeof( slave_reading.am2315_status ) );
    assignEntry( slave_reading.sd_status, "SLAVE", sizeof( slave_reading.sd_status ) );
    assignEntry( slave_reading.reading_status, reading_status.c_str(), sizeof( slave_reading.reading_status ) );

    sendData( Serial, (byte *)&slave_reading, sizeof( SENSOR_READING ) );

    while( !Serial.available() ); //block and wait for acknowledgement
    while( 1 ) 
    {
        while( receiveData( Serial, receive_buffer, buffer_index ) == TRANS_INCOMPLETE ); //block until we get the something
        bufferToCommand( receive_buffer, current_command );
        if( current_command.command[0] == ACKNOWLEDGE )
           break;
    }
}   

void loop()
{
    checkMaster();
    //Begin sampling
    sample();
}

void checkMaster( void )
{ 
    TRANS_TYPE transmission;
    GROUND_COMMAND current_command;

    if( !Serial.available() )
        return;

    if( ( transmission = receiveData( Serial, receive_buffer, buffer_index ) ) == TRANS_INCOMPLETE )
        return;
    else if( transmission == TRANS_COMMAND )
    {        
        if( bufferToCommand( receive_buffer, current_command ) )
        {
            reading_status = "CMD RECEIVED";
            if( current_command.command[0] == REQUEST_READING )
            {
                //Prepare to initiate slave readings
                downlinkToMaster();
            }
        }
        else
            reading_status = "BAD COMMAND";
    }
}

void downlinkToMaster( void )
{
   SENSOR_READING slave_reading = prepareReading();
   //Send the data
   sendData( Serial, (byte *)&slave_reading, sizeof( SENSOR_READING ) );
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
   
}

SENSOR_READING prepareReading( void )
{
    SENSOR_READING slave_reading;
    double so2_ppm;
    double no2_ppm;
    double o3_ppm;
    double temp;
    double humidity;
    double pressure;

    so2_ppm = sample_set.so2_total / sample_set.so2_count;
    no2_ppm = sample_set.no2_total / sample_set.no2_count;
    o3_ppm  = sample_set.o3_total  / sample_set.o3_count;

    temp = sample_set.temp_total / sample_set.temp_count;
    humidity = sample_set.humidity_total / sample_set.humidity_count;
    pressure = sample_set.pressure_total / sample_set.pressure_count;

    slave_reading.header[0] = '\x01';
    slave_reading.header[1] = '\x21';
    slave_reading.terminator[0] = '\r';
    slave_reading.terminator[1] = '\n';
    assignEntry( slave_reading.time, C_TIME(), sizeof( slave_reading.time) );
    assignEntry( slave_reading.bank, "2", sizeof( slave_reading.bank) );
    assignEntry( slave_reading.so2_reading, String( so2_ppm ).c_str(), sizeof( slave_reading.so2_reading ) );
    assignEntry( slave_reading.no2_reading, String( no2_ppm ).c_str(), sizeof( slave_reading.no2_reading ) );
    assignEntry( slave_reading.o3_reading, String( o3_ppm).c_str(), sizeof( slave_reading.o3_reading ) );
    assignEntry( slave_reading.temp_reading, String( temp ).c_str(), sizeof( slave_reading.temp_reading ) );
    assignEntry( slave_reading.extt_reading, "SLAVE", sizeof( slave_reading.extt_reading ) );
    assignEntry( slave_reading.pressure_reading, String( pressure ).c_str(), sizeof( slave_reading.pressure_reading ) );
    assignEntry( slave_reading.humidity_reading, String( humidity ).c_str(), sizeof( slave_reading.humidity_reading ) );
    assignEntry( slave_reading.pressure_reading, "0.00", sizeof( slave_reading.pressure_reading ) );
    assignEntry( slave_reading.humidity_reading, "0%", sizeof( slave_reading.humidity_reading ) );
    assignEntry( slave_reading.ext_humidity_reading, "SLAVE", sizeof( slave_reading.ext_humidity_reading ) );
    assignEntry( slave_reading.pump_status, "SLAVE", sizeof( slave_reading.pump_status ) );
    assignEntry( slave_reading.bme_status, bme_status.c_str(), sizeof( slave_reading.bme_status ) );
    assignEntry( slave_reading.am2315_status, "SLAVE", sizeof( slave_reading.am2315_status ) );
    assignEntry( slave_reading.sd_status, "SLAVE", sizeof( slave_reading.sd_status ) );
    assignEntry( slave_reading.reading_status, reading_status.c_str(), sizeof( slave_reading.reading_status ) );
}



