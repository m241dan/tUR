/*
 * HASP GOAT BABI: Pressure Test
 * Author: Daniel R. Koris
 * Goal of Program: Take barometric pressure and temperature reading at specified intervals
 * Required Components: Arduino Mega, Adafruit BME280, Adafruit MicroSD Card Breakout Board
 *                      Trustee MicroSD Card
 */

#include <SPI.h>
#include <SD.h>
#include <Adafruit_BME280.h>

/****************************
 * Config / Globals Section *
 ****************************/

//Dynamic Pins
#define BME_PIN	53
#define SD_PIN 49

//Sample Rate
#define SAMPLE_RATE 1 		//In Hz, this basically just sets our delay between readings

Adafruit_BME280 bme( BME_PIN );		//Handler for the BME sesnor, setup for hardwire SPI using an software CS pin
File hasp_log;				//This is the handle for the file we will open
float temperature, pressure, humidity;	//In a regular program, this would urk me, but I don't want the stack to reallocate everytime it loops
int time_since_start;
int file_iteration;
String buf;
String log_name;
void setup()
{
   file_iteration = 0;

   Serial.begin( 9600 );		//standard setup for baudrate
   while( !Serial );  			//waiting for serial to connect (not really needed when not hooked to laptop)
   
   if( !SD.begin( SD_PIN ) ) 		//this uses the static implementation of the SD library
   {
      Serial.println( "Initialization of SD failed." );
      return;
   }
   Serial.println( "SD Initialized." );

   if( !bme.begin() )			//This initializes the sensor with default x16 sampling rates, no high-pass filter, normal mode, .5 delay between readings
   {
      Serial.println( "Initialization of BME280 failed." );
      return;
   }
   Serial.println( "BME280 initialized" );

   time_since_start = 0;
   log_name = getNextFile();
}

void loop()
{
   delay( SAMPLE_RATE * 1000 );    //Delaying basically one second because I don't want to overflow the SD and the BME usually needs a bit of additional time to "warm" up

   hasp_log = SD.open( log_name.c_str(), FILE_WRITE );
      
   time_since_start += 1;		//This is a weak tracker of time since the program started
   temperature = 0.0;			//Since these are globals, let's make sure we zero them before we use them, incase the sensor fails to read
   humidity = 0.0;			//ibid
   pressure = 0.0;			//ibid
					//A sample program divided this reading by 100.0F, so I am too
   pressure = bme.readPressure() / 100.0F;
   temperature = bme.readTemperature();
   humidity = bme.readHumidity();


   buf = String( time_since_start ) + " " + String( pressure ) + " " + String( temperature ) + " " + String( humidity );

   hasp_log.println( buf.c_str() );
   hasp_log.close();
}

String getNextFile()
{
   String file_name;
   
   //This loop will basically find us a free file appended 1-2.1 billion, which should be plenty
   file_name = "run" + String( file_iteration ) + ".txt";
   for( file_iteration = 1; SD.exists( file_name ); file_iteration++ )
      file_name = "run" + String( file_iteration ) + ".txt";

   return file_name;
}

