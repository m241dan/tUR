#include "Arduino.h"
#include "HardwareSerial.h"
#include "hasp_types.h"
#include <SD.h>

void sendData( byte *data, int length )
{
    for( int x = 0; x < length; x++ )
    {
        Serial.write( *data );
        data++;
    }
}

//this function is specifically designed to fill in our readings entries with proper space padding and terminating each with a ','
void assignEntry( char *dst, const char *src, int length )
{
    for( int x = 0; x < length; x++ )
    {
        //value at src and put it into dst and then increment
        *dst = *src;
        dst++;
        src++;
        //if we are at the end of src (meaning always send '\0' terminated c strings as src) fill with spaces
        if( *src == '\0' )
        {
            for( int y = 0; y < ( (length-1) - x ); y++ )
            {
               *dst = ' ';
               dst++;
            }
            break;
        }
    }
    //terminate with ','
    *(dst-1) = ',';
}


String getNextFile( String name )
{
   String file_name;
   int file_iteration = 1;

   //This loop will basically find us a free file appended 1-2.1 billion, which should be plenty
   file_name = name + String( file_iteration ) + ".txt";
   for( ; SD.exists( file_name ); file_iteration++ )
      file_name = name + String( file_iteration ) + ".txt";

   return file_name;
}
