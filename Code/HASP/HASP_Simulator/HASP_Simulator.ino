/*
 * HASP Gondola: Simulator
 * Author: Daniel R. Koris, Erick Rameriz
 * Goal of the Program: The goal of this program is to simulate the HASP Gondola.
 * Required Components: Two Arduinos wired through Rx and Tx, 1200 Baud rate, Serial Uplink and Downlink. 
 */

#include <SD.h>
#include <SPI.h>


//Telemetry Rate
#define TELE_RATE ( 1000 * 60 )
#define SD_PIN 10

//GPS Telemetry
static String GPS = "";

//Global Variables
unsigned long long time_schedule;
String log_name;

void setup()
{
    //Begin Serial Communication
    Serial.begin( 1200 );
    while( !Serial );

    Serial1.begin( 9600 );
    while( !Serial1 );

    time_schedule = 0;
    log_name = getNextFile();    
}

void loop()
{
    /*if( ( time_scedule + TELE_RATE ) < millis() )
    {
        //Send telemetry data every minute
        sendTelemetry(gps, time_schedule)
    }
    else()
    {*/
        //Check for received data
        checkReceiveData();    
    //}
}

void checkReceiveData()
{
    File sim_log;
    if( Serial.available() )
    {
        if( !( sim_log = SD.open( log_name, FILE_WRITE ) ) )
        {
            Serial1.println( "File cannot be opened." );
            return;
        }
        while( Serial.available() )
            sim_log.write( Serial.read() );
        sim_log.close();
    }
}

String getNextFile()
{
   String file_name;
   int file_iteration = 1;
   
   //This loop will basically find us a free file appended 1-2.1 billion, which should be plenty
   file_name = "simulatorData   " + String( file_iteration ) + ".txt";
   for( ; SD.exists( file_name ); file_iteration++ )
      file_name = "run" + String( file_iteration ) + ".txt";

   return file_name;  
}
/*
void sendTelemetry(String tel, long long time_schedule)
{
    
    Serial.write("TEL: " + tel + " Tim: " + time_schedule);
}*/
