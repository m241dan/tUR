#include <SD.h>
#include "goat_funcs.h"

void sendData( HardwareSerial &serial, byte *data, int length )
{
    for( int x = 0; x < length; x++ )
    {
        serial.write( *data );
        data++;
    }
}

//this function is specifically designed to fill in our readings entries with proper space padding and terminating each with a ','
void assignEntry( char *dst, const char *src, int length, bool from_uplink = false )
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
    if( !from_uplink )
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

TRANS_TYPE receiveData( HardwareSerial &serial, byte (&buffer)[256], unsigned int &index, SENSOR_READING *reading, GROUND_COMMAND *com, GTP_DATA *gtp )
{
    TRANS_TYPE type = TRANS_INCOMPLETE;
    while( serial.available() )
    {
        byte c = serial.read();

        //we're receiving the start of a new transmission, whatever was in there is no longer relevant
        if( c == '\x1' )
        {
            memset( &buffer[0], 0, 256 );
            index = 0;
        }

        //store what we got
        buffer[index++] = c;

        //check if its a command or gtp
        if( c == '\x0A' && buffer[index-2] == '\x0D' && buffer[index-3] == '\x03' )
        {
            if( buffer[1] == '\x02' && com ) //then it's a command
            {
                assignEntry( com->checksum, &buffer[2], 1, true );
                assignEntry( com->command, &buffer[3], 2, true);
                //executeCMD();
                type = TRANS_COMMAND;
            }
            else if( buffer[1] == '\x30' && gtp ) //then it's a gps
            {
                assignEntry( gtp->data, &buffer[2], sizeof( gtp->data ), true );
                //parseGTP();
                type = TRANS_GTP;
            }
        }
        //check if its data our data (like, from Slave)
        else if( c == '\n' && buffer[index-2] == '\r' && reading )
        {
             //just going to do memcpy here because otherwise its a million assign statements....
             memcpy( &reading->time[0], &buffer[2], sizeof( SENSOR_READING ) - 4 );
             type = TRANS_DATA;
        }
        //something has gone terribly wrong, just reset
        else if( index == 256 )
        {
            memset( &buffer[0], 0, 256 );
            index = 0;
        }

    }
    return type;
}

//simply a wrapper function
void sendCommand( HardwareSerial &serial, GROUND_COMMAND &com )
{
   sendData( serial, (byte *)&com, sizeof( com ) );
}
