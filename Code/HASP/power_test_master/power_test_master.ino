/*
 *  HASP GOAT: Power Test
 *  Author: Daniel R. Koris
 *  Goal of Program: Run all systems simultaneous in order to get an accurate
 *                    amperage draw from the system at 30 volts.
 *  Required Components: Arduino Mega Master, Adafruit BME280,
 *                       Adafruit MicroSD Card Breakout x2, Spece SO2 Analog,
 *                       Spec 03 Analog, Spec NO2 Analog, Adafruit SI5351 Oscillator, 
 *                       AM2315, Peltier Devices, and Resistance Heaters
 */

/* 
 * libraries
 */
 
#include <SPI.h>                    //for general SPI stuff
#include <SD.h>                     //for the SD card, duh
#include <Wire.h>                   //For the dreaded I2C stuff
#include <Adafruit_SI5351.h>        //Oscillator
#include <Adafruit_BME280.h>        //BME280 Temp/Humidity/Pressure sensor
#include <Adafruit_AM2315.h>        //AM2315 External Temp/Humidity sensor
#include "Spec.h"                   //For the Spec sensors

/*
 * defaults
 */
 
#define SO2_GAS 1                   //Pins for Spec Sensors
#define SO2_REF 2
#define SO2_TMP 3
#define NO2_GAS 4
#define NO2_REF 5
#define NO2_TMP 6
#define O3_GAS  7
#define O3_REF  8
#define O3_TMP  9

#define SD_ONE_CS   49
#define SD_TWO_CS   48

#define BME_CS      47

// resistors/peltiers


/*
 * globals
 */

Spec SO2_sensor( SPEC_SO2, SO2_GAS, SO2_REF, SO2_TMP );
Spec O3_sensor( SPEC_O3, O3_GAS, O3_REF, O3_TMP );
Spec NO2_sensor( SPEC_NO2, NO2_GAS, NO2_REF, NO2_TMP );

SDClass SD1;
SDClass SD2;

Adafruit_BME280 bme( BME_CS );
Adafruit_AM2315 am2315;
Adafruit_SI5351 hasp_clock = Adafruit_SI5351();
File hasp_log;
String log_name;
File hasp_log_bak;
String log_name_bak;

void setup()
{
    initSerials();
    if( !initSDs() )
        return;
    if( !initSensors() )
        return;
    /*
     * I am writing this note to myself that we will need an error msg packet 
     * that we can send to the ground. However, we will also have LEDs? But if
     * we are either plugged into HASP gondola or our HASP simulator, we will
     * get the feedback. And, I imagine in most circumstances we will be plugged
     * into our HASP simulator. Except for Vacuum tests... so we will need the
     * the LEDs still. I don't know why I'm so against the LEDs.
     */
    reportError( "Setup complete." );
}

void loop()
{
    String reading;
    if( !( hasp_log = SD1.open( log_name.c_str(), FILE_WRITE ) ) )
    {
        reportError( "Loop(): Failed to open hasp_log." );
        return;
    }
    if( !( hasp_log_bak = SD2.open( log_name_bak.c_str(), FILE_WRITE ) ) )
    {
        reportError( "Loop(): Failed to open hasp_log_bak." );
        return;
    }

    reading += " " + String( bme.readPressure() / 100.0F );
    reading += " " + String( bme.readTemperature() );
    reading += " " + String( bme.readHumidity() );
    reading += " " + SO2_sensor.generateReading();
    reading += " " + NO2_sensor.generateReading();
    reading += " " + O3_sensor.generateReading();
    reading += " " + String( am2315.readHumidity() );
    reading += " " + String( am2315.readTemperature() );

    hasp_log.println( reading.c_str() );
    hasp_log_bak.println( reading.c_str() );

    hasp_log.close();
    hasp_log_bak.close();
    

}

bool initSerials() //the fact that this is a boolean function is purely symbolic at the moment
{
    Serial.begin( 1200 );           //baudrate for the downlink/uplink to HASP
    while( !Serial );               //block until it's ready

    Serial1.begin( 9600 );          //baudrate for the connection to the Slave (we can turn this up should it be necessary)
    while( !Serial1 );              //block until it's ready

    return true;
}

bool initSDs()
{    
    if( !SD1.begin( SD_ONE_CS ) ) 
    {
        reportError( "SD one failed to initialize." );
        return false;
    }

    log_name = getNextFileName( &SD1 );

    if( !SD2.begin( SD_TWO_CS ) )
    {
        reportError( "SD two failed to initialize." );
        return false;
    }

    log_name_bak = getNextFileName( &SD2 );
    
    return true;
}

bool initSensors()
{
    err_t err_msg;
    
    if( !bme.begin() )
    {
        reportError( "BME280 failed to initialize." );
        return false;
    }

    if( !am2315.begin() )
    {
        reportError( "AM2315 failed to initialize." );
        return false;
    }

    if( ( err_msg = hasp_clock.begin() ) != ERROR_NONE )
    {
        reportError( "Hasp clock failed to initialize with message code: " + String( err_msg ) );
        return false;
    }

    hasp_clock.setupPLLInt(SI5351_PLL_A, 36);
    //Divider 4/8/16 etc
    hasp_clock.setupMultisynthInt(0, SI5351_PLL_A, SI5351_MULTISYNTH_DIV_8);
    hasp_clock.enableOutputs(true);
    
    return true;
}

//This is sort of just a placeholder for a real error messaging system
void reportError( String error_msg )
{
    Serial.println( error_msg );
}

String getNextFileName( SDClass *sd )
{
    String file_name;
    int file_iteration = 0;
    
   //This loop will basically find us a free file appended 1-2.1 billion, which should be plenty
   file_name = "run" + String( file_iteration ) + ".txt";
   for( file_iteration = 1; sd->exists( file_name ); file_iteration++ )
      file_name = "run" + String( file_iteration ) + ".txt";

   return file_name;
}

