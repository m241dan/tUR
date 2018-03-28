#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include "Adafruit_ADXL345_U.h"
#include <SoftwareSerial.h>
#include <StandardCplusplus.h>
#include <vector>
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
#define SOFT_TX 34
#define SOFT_RX 36
#define SD_PIN 10

RTC_PCF8523 rtc;

std::vector<int> BME_IDs = { BME_1, BME_2, BME_3, BME_4, BME_5, BME_6 };
std::vector<Adafruit_BME280> BMEs;

Adafruit_BME280 bme1( BME_1 );
Adafruit_BME280 bme2( BME_2 );
Adafruit_BME280 bme3( BME_3 );
Adafruit_BME280 bme4( BME_4 );
Adafruit_BME280 bme5( BME_5 );
Adafruit_BME280 bme6( BME_6 );

SoftwareSerial cameraconnection = SoftwareSerial(SOFT_RX, SOFT_TX);

Adafruit_ADXL345_Unified accel( 12345 );

String error_log;

File data_file;

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

    for( int i = 0; i < BME_IDs.size(); i++ )
    {
        BMEs.at(i) = Adafruit_BME280( BME_IDs[i] );
        if( !BMEs.at(i).begin() )
        {
            String error_msg = "BME" + String( (i+1) ) + "is not working.\r\n";
            Serial.println( error_msg );
            error_log += error_msg;
        }
    }
        
    if( !accel.begin() )
    {
        String error_msg = "Accel not working.\r\n";
        Serial.println( error_msg );
        error_log += error_msg;
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
    Serial.println( "Error Log: " + error_log );
    
}

void loop()
{
    /* this will prepend all written data */
    String output_prepend = generateOutputPrepend();
    /* this will prepend all files saved */             
    String file_prepend = generateFilePrepend();
    

    /* if we have errors, write them to their own log */
    if( error_log != "" )
    {
        writeErrorLog( output_prepend, file_prepend );
    }
    else
    {
        Serial.println( "Operating without any errors." );
    }
    
    sensors_event_t event;
    accel.getEvent( &event );

    /* read data */
    std::vector<double> temperature_readings;
    std::vector<double> humidity_readings;
    std::vector<double> pressure_readings;

    temperature_readings.clear();
    humidity_readings.clear();
    pressure_readings.clear();
    
    for( int i = 0; i < BMEs.size(); i++ )
    {
        Adafruit_BME280 &cur_bme = BMEs.at(i);
        temperature_readings.at(i) = cur_bme.readTemperature();
        humidity_readings.at(i) = cur_bme.readHumidity();
        pressure_readings.at(i) = cur_bme.readPressure() / 100.0F;
        
    }
    /* write data */
    /* check pressure */
    /* flip pump */

    delay(1000);
}

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

void writeErrorLog( String output_prepend, String file_prepend )
{
    File error_file;
    
    String error_file_name = file_prepend + "err";
    error_file = SD.open( error_file_name.c_str(), FILE_WRITE );
    if( error_file )
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
