/*
 * HASP GOAT BABI: Pressure Test
 * Author: Daniel R. Koris, Erick Rameriz
 * Goal of Program: Take barometric pressure and temperature reading at specified intervals
 * Required Components: Arduino Mega, Adafruit BME280, Adafruit MicroSD Card Breakout Board
 *                      Trustee MicroSD Card
 */

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

/****************************
 * Config / Globals Section *
 ****************************/

//Dynamic Pins
#define BME_PIN	10
#define SD_PIN 9

//Sample Rate
#define SAMPLE_RATE 1000 		//In Hz, this basically just sets our delay between readings

Adafruit_BME280 bme( BME_PIN );		//Handler for the BME sesnor, setup for hardwire SPI using a software CS pin
int iteration;
unsigned long long time_schedule;
String log_name;


void setup()
{
   Serial.begin( 9600 );		//standard setup for baudrate
   while( !Serial );  			//waiting for serial to connect (not really needed when not hooked to laptop)

   if(!initSensors()){
    Serial.println("Error initializng sensors");
    return;
   }

   time_schedule = 0;
   iteration = 0;
   log_name = getNextFile();
}

void loop()
{
    if( ( time_schedule + SAMPLE_RATE ) < millis() )
    {
        takeReadings( true, true );
        time_schedule = millis();
    }
 //   Serial.println( "Branding..." );
}

void takeReadings( bool sd_write, bool blu_write )
{
    float pressure, temperature, humidity;
    String buf;
    
    pressure = temperature = humidity = 0.0;
    iteration++;
    
    pressure = bme.readPressure() / 100.0F;
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();

    buf = String( iteration ) + " " + String( pressure ) + " " + String( temperature ) + " " + String( humidity );

    if( sd_write )
    {
        File hasp_log = SD.open( log_name.c_str(), FILE_WRITE );
        hasp_log.println( buf );
        hasp_log.close();        
    }
    if( blu_write )
        Serial.println( buf );
}

String getNextFile()
{
   String file_name;
   int file_iteration = 0;
   
   //This loop will basically find us a free file appended 1-2.1 billion, which should be plenty
   file_name = "run" + String( file_iteration ) + ".txt";
   for( file_iteration = 1; SD.exists( file_name ); file_iteration++ )
      file_name = "run" + String( file_iteration ) + ".txt";

   return file_name;
}

//Initialize the sensors
boolean initSensors()
{
   if( !SD.begin( SD_PIN ) )     //this uses the static implementation of the SD library
   {
      Serial.println( "Initialization of SD failed." );
      return false;
   }

   if( !bme.begin() )     //This initializes the sensor with default x16 sampling rates, no high-pass filter, normal mode, .5 delay between readings
   {
      Serial.println( "Initialization of BME280 failed." );
      return false;
   }
   return true;
}

