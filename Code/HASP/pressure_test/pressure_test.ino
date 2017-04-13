/*
 * HASP GOAT BABI: Pressure Test
 * Author: Daniel R. Koris
 * Goal of Program: Take barometric pressure and temperature reading at specified intervals
 * Required Components: Arduino Mega, Adafruit BME280, Adafruit MicroSD Card Breakout Board
 *                      Trustee MicroSD Card
 */

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
<<<<<<< HEAD
#include <Custom/Adafruit_BME280.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#include <Adafruit_SI5351.h>
=======
#include <Adafruit_BME280.h>
>>>>>>> 9feda545fc61d2e1f4a48c01aa6fee6e0ffbbc97

/****************************
 * Config / Globals Section *
 ****************************/

//Dynamic Pins
#define BME_PIN	53
#define SD_PIN 49
#define BLE_CS 35





//Sample Rate
#define SAMPLE_RATE 1 		//In Hz, this basically just sets our delay between readings



Adafruit_BME280 bme( BME_PIN );		//Handler for the BME sesnor, setup for hardwire SPI using an software CS pin
//Initialzed the clock
Adafruit_SI5351 haspClock = Adafruit_SI5351();
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

   if(!initSensors()){
    Serial.println("Error initializng sensors");
    return;
   }
   
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
   //Send the data check for bytes Max is 24 bytes
   ble.println( buf.c_str() );
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

//Initialize the sensors
boolean initSensors(){
  boolean stat = true;

  if( !SD.begin( SD_PIN ) )     //this uses the static implementation of the SD library
   {
      Serial.println( "Initialization of SD failed." );
        stat = false;
   }

   if( !bme.begin() )     //This initializes the sensor with default x16 sampling rates, no high-pass filter, normal mode, .5 delay between readings
   {
      Serial.println( "Initialization of BME280 failed." );
      stat = false
      return;
   }

   if (haspClock.begin() != ERROR_NONE)
  {
    
    Serial.print("Ooops, no Si5351 detected ... Check your wiring or I2C ADDR!");
    stat = false;
    return;
  }
  else{
      
  Serial.println("Set PLLA to 900MHz");
  haspClock.setupPLLInt(SI5351_PLL_A, 36);
  Serial.println("Set Output #0 to 112.5MHz");  
  //Divider 4/8/16 etc
  haspClock.setupMultisynthInt(0, SI5351_PLL_A, SI5351_MULTISYNTH_DIV_8);
  haspClock.enableOutputs(true);
  }
   
}

