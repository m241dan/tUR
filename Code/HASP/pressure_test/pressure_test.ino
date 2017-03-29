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
#define SD_PIN 54

//Sample Rate
#define SAMPLE_RATE = 1; 		//In Hz, this basically just sets our delay between readings

Adafruit_BME280 bme( BME_PIN );		//Handler for the BME sesnor, setup for hardwire SPI using an software CS pin
File log;				//This is the handle for the file we will open


void setup()
{
   int file_iteration = 1;

   Serial.begin( 9600 );		//standard setup for baudrate
   while( !Serial );  			//waiting for serial to connect (not really needed when not hooked to laptop)

   if( !SD.begin( SD_PIN ) ) 		//this uses the static implementation of the SD library
   {
      Serial.println( "Initialization of SD failed." );
      return;
   }

					//This loop will basically find us a free file appended 1-2.1 billion, which should be plenty
   for( int x = 1; SD.exists( String( "Test_Run_" + x ) ); x++ );

   if( !bme.begin() )			//This initializes the sensor with default x16 sampling rates, no high-pass filter, normal mode, .5 delay between readings
   {
      Serial.println( "Initialization of BME280 failed." );
      return;
   }


}

void loop()
{

}
