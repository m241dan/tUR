#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include "Adafruit_ADXL345_U.h"
#include <SoftwareSerial.h>
#include <Adafruit_VC0706.h>
#include "goat_funcs.h"

#define BME_1 22
#define BME_2 24
#define BME_3 26
#define BME_4 28
#define BME_5 30
#define BME_6 32
#define SOFT_TX 34
#define SOFT_RX 36
#define SD_PIN 10
#define NUM_BMEs 6
#define CAM_INTERVAL 30 //in seconds
#define PUMP_ON_PRESSURE 100.00f
#define PUMP_OFF_PRESSURE 300.00f

typedef struct data_table
{
  double temp[NUM_BMEs];
  double hum[NUM_BMEs];
  double pres[NUM_BMEs];
  double accel_x;
  double accel_y;
  double accel_z;
} DataTable;

/* Sensors */
RTC_PCF8523 rtc;
Adafruit_BME280 BMEs[NUM_BMEs] = { Adafruit_BME280( BME_1 ), Adafruit_BME280( BME_2 ), Adafruit_BME280( BME_3 ), Adafruit_BME280( BME_4 ), Adafruit_BME280( BME_5 ), Adafruit_BME280( BME_6 ) };
Adafruit_ADXL345_Unified accel( 12345 );
SoftwareSerial camera_connection = SoftwareSerial(SOFT_RX, SOFT_TX);
Adafruit_VC0706 cam = Adafruit_VC0706( &camera_connection );

String error_log;
long long last_photo;
bool photo_to_save;
bool pump_on;

void setup()
{
    Serial.begin( 57600 );
    while ( !Serial );

    /* init these globals */
    error_log = "";
    pump_on = false;
    last_photo = millis();
    photo_to_save = false;

    if ( !rtc.begin() )
    {
        String error_msg = "RTC not working.\r\n";
        Serial.print( error_msg );
        error_log += error_msg;
    }
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    for ( int i = 0; i < NUM_BMEs; i++ )
    {
        if ( !BMEs[i].begin() )
        {
            String error_msg = "BME" + String( (i + 1) ) + "is not working.\r\n";
            Serial.print( error_msg );
            error_log += error_msg;
        }
    }

    if ( !accel.begin() )
    {
        String error_msg = "Accel not working.\r\n";
        Serial.print( error_msg );
        error_log += error_msg;
    }
    else
    {
        accel.setRange( ADXL345_RANGE_16_G );
        accel.setDataRate( ADXL345_DATARATE_50_HZ );
    }

    if ( !cam.begin() )
    {
        String error_msg = "Camera failed to initialize.\r\n";
        Serial.print( error_msg );
        error_log += error_msg;
    }
    else
    {
        cam.setImageSize( VC0706_640x480 );
    }

    if ( !SD.begin( SD_PIN ) )
    {
        Serial.println( "SD did not init" );
    }
    Serial.println( "Error Log: " + error_log );
}

void loop()
{
    /* this will prepend all written data */
    String output_prepend = generateOutputPrepend();

    /* this will prepend all files saved */
    String file_prepend = generateFilePrepend();

    /* read data */
    DataTable data;
    readBMEs( data );
    readAccel( data );
    if ( shouldTakePhoto == true )
        takeImg();

    /* write data */
    if( photo_to_save )
        saveImg();

    /* if we have errors, write them to their own log */
    if ( error_log != "" )
    {
        writeErrorLog( output_prepend, file_prepend );
    }
    else
    {
        Serial.println( "Operating without any errors." );
    }

    /* check pressure */
    /* flip pump */

    delay(1000);
}

/*
 * File / Output Convention Functions
 * Written by Daniel R. Koris
 */

String generateOutputPrepend()
{
    DateTime now = rtc.now();
    String prepend = "H:" + String( now.hour() ) +
                     "M:" + String( now.minute() ) +
                     "S:" + String( now.second() ) + ":";
    return prepend;
}

String generateFilePrepend()
{
    DateTime now = rtc.now();
    String file_prepend = String( now.month() ) + ":" +
                          String( now.day() );

    return file_prepend;
}

/*
 * Error Log Writing Function
 * Written by Daniel R. Koris
 */
void writeErrorLog( String output_prepend, String file_prepend )
{
    File error_file;
    String error_file_name = file_prepend + "err";

    error_file = SD.open( error_file_name.c_str(), FILE_WRITE );
    if ( error_file )
    {
        String to_write = output_prepend + error_log;
        error_file.println( to_write.c_str( ));
        error_file.close();
        error_log = "";
    }
    else
    {
        Serial.println( "Failed to open error log." );
    }
}

/*
 * Writing our Data Functions
 * Written by Daniel R. Koris
 */
void writeData( DataTable &table)
{

}

/*
 * Reading out Adafruit Sensors
 * Written by Daniel R. Koris
 */
void readBMEs( DataTable &table)
{
    memset( &table.temp[0], 0, sizeof( table.temp ) );
    memset( &table.hum[0], 0, sizeof( table.hum ) );
    memset( &table.pres[0], 0, sizeof( table.pres ) );

    for ( int i = 0; i < NUM_BMEs; i++ )
    {
        Adafruit_BME280 &cur_bme = BMEs[i];
        table.temp[i] = cur_bme.readTemperature();
        table.hum[i] = cur_bme.readHumidity();
        table.pres[i] = cur_bme.readPressure() / 100.0F;
    }
}

void readAccel( DataTable &table)
{
    sensors_event_t event;

    accel.getEvent( &event );

    table.accel_x = event.acceleration.x;
    table.accel_y = event.acceleration.y;
    table.accel_z = event.acceleration.z;
}

/*
 * Camera Specific Functions
 * Written by Daniel R. Koris
 */
bool shouldTakePhoto()
{
    bool take_photo = false;

    if ( ( last_photo - millis() ) >= ( CAM_INTERVAL * 1000 ) )
        take_photo = true;

    return take_photo;
}

void takeImg()
{
    if ( !cam.takePicture() )
    {
        String error_msg = "Camera failed to take an image.\r\n";
        Serial.print( error_msg );
        error_log += error_msg;
        photo_to_save = false;
    }
    photo_to_save = true;
}

void saveImg()
{
    File img_file;
    String img_file_name = getNextFile( "IMG", ".jpg" );

    
    img_file = SD.open( img_file_name.c_str(), FILE_WRITE );
    if( img_file )
    {
        uint16_t len = cam.frameLength();

        while( len > 0 )
        {
            uint8_t *buffer;
            uint8_t bytes_to_read = min( 32, len );
            buffer = cam.readPicture( bytes_to_read );

            img_file.write( buffer, bytes_to_read );
            len -= bytes_to_read;
        }
        photo_to_save = false;
        img_file.close();
    }
    else
    {
        String error_msg = "Could not open file to write an image.\r\n";
        Serial.print( error_msg );
        error_log += error_msg;
    }
}
