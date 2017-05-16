/*
 * HASP GOAT: Spec SO2 Sensor Test
 * Author: Daniel R. Koris
 * Goal of Program: To run the Spec SO2 sensor and log the data on an SD. Bluetooth component
 *                  is optional.
 * Required Components: Arduino (any), Adafruit SD Breakout, Non-BLE Bluetooth Device (wired in Serial 0),
 *                      and Spec SO2 Sensor. 
 */

/*
 * libraries
 */
#include <SPI.h>
#include <SD.h>
#include <Spec.h>

/*
 * globals/config
 */
#define SD_PIN 48

String log_name;
File SO2_log;
Spec SO2_sensor( SPEC_SO2, A0, A1, A2, 43.45 );
unsigned long last_blu = 0UL;

void setup()
{
    Serial.begin( 9600 );
    while( !Serial );

    if( !SD.begin( SD_PIN ) )
    {
        Serial.println( "SD failed to initialize." );
        return; 
    }
    log_name = getNextFileName();
    delay( 2000 );
    Serial.println( "Intialization Success." );
}

void loop()
{
    String reading;
    if( !( SO2_log = SD.open( log_name.c_str(), FILE_WRITE ) ) )
    {
        Serial.println( "Loop(): Failed to open Log." );
        return; 
    }

    reading = SO2_sensor.generateRawReading( ' ', false, false);
    
    SO2_log.println( reading.c_str() );
    SO2_log.close();

    //bluetooth seems to limited at one per second
    if( ( last_blu + 1000 ) <= millis() )
    {
        last_blu = millis();
        Serial.println( reading );
    }
    delay( 1000 );
}

String getNextFileName()
{
    String file_name;
    int file_iteration = 0;

    //This loop will basically find us a free file append up to 2.1 billion
    file_name = "SO2RUN" + String( file_iteration ) + ".txt";
    for( ; SD.exists( file_name ); file_iteration++ )
        file_name + "SO2RUN" + String( file_iteration ) + ".txt";

    return file_name;
}

