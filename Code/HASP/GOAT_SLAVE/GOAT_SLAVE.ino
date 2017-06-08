
#include "slave_globals.h"
#include "goat_slave_funcs.h"
#include "goat_funcs.h"

Spec so2( SPEC_SO2, A0, A1, A2, 43.54 );
Spec no2( SPEC_NO2, A3, A4, A5, 43.53 );
Spec o3( SPEC_O3, A6, A7, A8, 43.56 );
Adafruit_BME280 bme( BME_PIN );
GROUND_COMMAND current_command;
SENSOR_READING slave_reading;
byte receive_buffer[MAX_BUF];
unsigned int buffer_index;
String reading_status = "GOOD";
String bme_status;

//Data set struct that holds our data
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
    
}DATA_SET;

DATA_SET sample_set;

void setup()
{
    //Setup and initialize sensors
    setupSlaveSerials();
    setupSlaveGlobals();
    setupSlaveSensors();

    delay( 10000 );
    assignEntry( slave_reading.time, C_TIME(), sizeof( slave_reading.time ) );
    sendData( Serial, (byte *)&slave_reading, sizeof( slave_reading ) );

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
    checkMaster();
    //Begin sampling
    sample();
}

void checkMaster( void )
{ 
    TRANS_TYPE transmission;
    if( !Serial.available() )
        return;

    if( ( transmission = receiveData( Serial1, receive_buffer, buffer_index, nullptr, &current_command, nullptr ) ) == TRANS_INCOMPLETE )
        return;
    else if( transmission == TRANS_COMMAND )
    {        
        if( current_command.command[0] == REQUEST_READING )
        {
            //Prepare to initiate slave readings
            downlinkToMaster();
        }
    }
}

void downlinkToMaster( void )
{
   prepareReading();
   //Send the data
   sendData( Serial, (byte *)&slave_reading, sizeof( slave_reading ) );
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

void prepareReading( void )
{
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

    memset( &sample_set, 0 , sizeof( sample_set ) );

    assignEntry( slave_reading.time, C_TIME(), sizeof( slave_reading.time) );
    assignEntry( slave_reading.bank, "2", sizeof( slave_reading.bank) );
    assignEntry( slave_reading.so2_reading, String( so2_ppm ).c_str(), sizeof( slave_reading.so2_reading ) );
    assignEntry( slave_reading.no2_reading, String( no2_ppm ).c_str(), sizeof( slave_reading.no2_reading ) );
    assignEntry( slave_reading.o3_reading, String( o3_ppm).c_str(), sizeof( slave_reading.o3_reading ) );
    if( bme_status != "BIFD" )
    {
        assignEntry( slave_reading.temp_reading, String( temp ).c_str(), sizeof( slave_reading.temp_reading ) );
        assignEntry( slave_reading.pressure_reading, String( pressure ).c_str(), sizeof( slave_reading.pressure_reading ) );
        assignEntry( slave_reading.humidity_reading, String( humidity ).c_str(), sizeof( slave_reading.humidity_reading ) );
        
    }

    assignEntry( slave_reading.reading_status, reading_status.c_str(), sizeof( slave_reading.reading_status ) );

    
    
}



