#include <SD.h>
#include "goat_funcs.h"

//this is a bit of a wrapper that is convenient for sending data over different serials
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

TRANS_TYPE receiveData( HardwareSerial &serial, byte (&buffer)[MAX_BUF], unsigned int &index )
{
    TRANS_TYPE type = TRANS_INCOMPLETE;
    while( serial.available() && type == TRANS_INCOMPLETE )
    {
        byte c = serial.read();

        //we're receiving the start of a new transmission, whatever was in there is no longer relevant
        if( c == '\x1' )
            resetBuffer( buffer, index );

        //store what we got
        buffer[index++] = c;
        //check if its a command or gtp
        if( c == '\x0A' && buffer[index-2] == '\x0D' && buffer[index-3] == '\x03' )
        {
            if( buffer[1] == '\x02' ) //then it's a command
                type = TRANS_COMMAND;
            else if( buffer[1] == '\x30' ) //then it's a gps
                type = TRANS_GTP;
        }
        //check if its data our data (like, from Slave)
        else if( c == '\n' && buffer[index-2] == '\r' )
            type = TRANS_DATA;
        //something has gone terribly wrong, just reset
        else if( index == MAX_BUF )
           resetBuffer( buffer, index );
    }
    return type;
}

bool bufferToReading( byte (&buffer)[MAX_BUF], SENSOR_READING &reading )
{
    bool data_good = false;
    /*
     * Using memset first here may be redundant...
     * Clear out the given reading
     */
    memset( &reading, 0, sizeof( SENSOR_READING ) );
    /*
     * Copy the buffer into reading raw
     */
    memcpy( &reading, &buffer[0], sizeof( SENSOR_READING ) );
    /*
     * Check the header and the terminators for data corruption
     */
    if( reading.header[0] == '\x1' && reading.header[1] == '\x21' &&
        reading.terminator[0] == '\r' && reading.terminator[1] == '\n' )
    {
       data_good = true;
    }

    return data_good;
}

bool bufferToCommand( byte (&buffer)[MAX_BUF], GROUND_COMMAND &com )
{
    bool com_good = false;
    /*
     * Using memset first here may be redundant...
     * Clear out the given com
     */
    memset( &com, 0, sizeof( GROUND_COMMAND ) );
    /*
     * Copy the buffer into reading raw
     */
    memcpy( &com, &buffer[0], sizeof( GROUND_COMMAND ) );
    /* 
     * Check the header and the terminators for data corruption
     */
    if( com.header[0] == '\x1' && com.header[1] == '\x2' &&
        com.terminator[0] == '\x3' && com.terminator[1] == '\xD' &&
        com.terminator[2] == '\xA' )
    {
        com_good = true;
    }

    return com_good;
}

void bufferToGTP( byte (&buffer)[MAX_BUF], GTP_DATA &gtp )
{
    bool gtp_good = false;
    /*
     * Using memset first here may be redundant...
     * Clear out the given gtp
     */
    memset( &gtp, 0, sizeof( GTP_DATA) );
    /*
     * Copy the buffer into reading raw
     */
    memcpy( &gtp, &buffer[0], sizeof( GTP_DATA ) );
    /*
     * Check the header and the terminators for data corruption
     */
    if( gtp.header[0] == '\x1' && gtp.header[1] == '\x30' &&
        gtp.terminator[0] == '\x3' && gtp.terminator[1] == '\xD' &&
        gtp.terminator[2] == '\xA' )
    {
       gtp_good = true;
    }
    return gtp_good;
}

void resetBuffer( byte (&buffer)[MAX_BUF], unsigned int &index )
{
   memset( &buffer[0], 0, MAX_BUF );
   index = 0;
}


/*
 * This will create and send a command with the given passed values
 */
void sendCommand( HardwareSerial &serial, unsigned char command )
{
   GROUND_COMMAND com;
   com.command[0] = command;
   sendData( serial, (byte *)&com, sizeof( com ) );
}
