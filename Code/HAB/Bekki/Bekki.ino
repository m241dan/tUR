#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include "Adafruit_ADXL345_U.h"

/* ADXL labels
 *  SDA/SDI/SDIO
 *  SDO
 *  SCL/SCLK
 */

/* Arduino Mega Hardware SPI Pins
 *  MOSI - 51
 *  MISO - 50
 *  SCK  - 52
 */
#define SOFTWARE_MOSI 37
#define SOFTWARE_MISO 35
#define SOFTWARE_SCK  33
#define BME_1 22
#define BME_2 24
#define BME_3 26
#define BME_4 28
#define BME_5 30
#define BME_6 32
#define ACCEL_1 34
#define ACCEL_2 36
#define SD_PIN 10

RTC_PCF8523 rtc;

Adafruit_BME280 bme1( BME_1 );
Adafruit_BME280 bme2( BME_2 );
Adafruit_BME280 bme3( BME_3 );
Adafruit_BME280 bme4( BME_4 );
Adafruit_BME280 bme5( BME_5 );
Adafruit_BME280 bme6( BME_6 );

Adafruit_ADXL345_Unified accel( 12345 );

String error_log;

File data_file;
File error_file;

void setup()
{
    Serial.begin( 57600 );
    while( !Serial );
    error_log = "";
    
    if( !rtc.begin() )
    {
        Serial.println( "RTC not working." );
        error_log += String( "RTC not working.\r\n" );
    }
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));    
    
    if( !bme1.begin() )
    {
        Serial.println( "BME1 not working" );
        error_log += String( "BME1 not working.\r\n" );
    }
    if( !bme2.begin() )
    {
        Serial.println( "BME2 not working" );
        error_log += String( "BME2 not working.\r\n" );
    }
    if( !bme3.begin() )
    {
        Serial.println( "BME3 not working" );
        error_log += String( "BME3 not working.\r\n" );
    }
    if( !bme4.begin() )
    {
        Serial.println( "BME4 not working" );
        error_log += String( "BME4 not working.\r\n" );
    }
    if( !bme5.begin() )
    {
        Serial.println( "BME5 not working" );
        error_log += String( "BME5 not working.\r\n" );
    }
    if( !bme6.begin() )
    {
        Serial.println( "BME6 not working" );
        error_log += String( "BME6 not working.\r\n" );
    }
        
    if( !accel.begin() )
    {
        Serial.println( "Accel not working." );
        error_log += String( "Accel not working.\r\n" );
    }
    else
    {
        accel.setRange( ADXL345_RANGE_16_G );
        accel.setDataRate( ADXL345_DATARATE_50_HZ );
    }

    if( !SD.begin( SD_PIN ) )
    {
        Serial.println( "SD did not init" );
    }
}

void loop()
{
    /* this will prepend all written data */
    DateTime now = rtc.now();
    String prepend = String( now.hour() ) + "/" +
                 String( now.minute() ) + "/" +
                 String( now.second() ) + ":";

    /* this will prepend all files saved */             
    String file_name_base = String( now.month() ) + ":" +
                            String( now.day() );

    /* if we have errors, write them to their own log */
    if( error_log != "" )
    {
       String error_file_name = file_name_base + "err";
       Serial.println( error_file_name );
       error_file = SD.open( error_file_name.c_str(), FILE_WRITE );
       if( error_file )
       {
            String to_write = prepend + error_log;
            error_file.println( to_write.c_str( ));
            error_file.close();
            error_log = "";
        }
        else
        {
            Serial.println( "Failed to open error log." );
        }
    }
    else
    {
        Serial.println( "Did not write any errors" );
    }
    
    String buf_file_name = file_name_base + "out";

    sensors_event_t event;
    accel.getEvent( &event );

    /* read data */
    /* write data */
    /* check pressure */
    /* flip pump */

    delay(1000);
}


//    Serial.print(now.hour(), DEC);
//    Serial.print(':');
//    Serial.print(now.minute(), DEC);
//    Serial.print(':');
//    Serial.print(now.second(), DEC);
//    Serial.println();
//
//    Serial.println( "Temp: " + String( bme1.readTemperature() ) + "C" );
//    Serial.println( "Pres: " + String( bme1.readPressure() / 100.0F ) + "hPA" );
//    Serial.println( "Humi: " + String( bme1.readHumidity() ) + "%" );
//
//    Serial.println( "X : " + String( event.acceleration.x ) + " Y: " +
//                    String( event.acceleration.y ) + " Z: " + String( event.acceleration.z ) );
