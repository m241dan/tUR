/*
 * Spoof the GOAT
 */

#include "hasp_types.h"

#define BUF_SIZE 256    //A bit excessive, but better safe than sorry
SENSOR_READING reading;
GROUND_COMMAND current_command;
GTP_DATA current_gtp;

unsigned long long time_schedule;
unsigned int index;

void setup()
{
    Serial.begin( 1200 );
    while( !Serial );

    time_schedule = 0;
    index = 0;
}

void loop()
{
    while( Serial.available() )
    {
        static byte receive_buffer[BUF_SIZE];
        byte c = Serial.read();

        if( c == '\x1' )
        {
            memset( &receive_buffer[0], 0, BUF_SIZE );
            index = 0;
        }

        receive_buffer[index++] = c;

        if( c == '\x0A' )
        {
            if( receive_buffer[1] == '\x02' ) //then it's a command
            {
                Serial.println( "Command received" );
                memset( &current_command.command[0], 0, sizeof( current_command.command ) );
                assignEntry( current_command.checksum, &receive_buffer[2], sizeof( current_command.checksum ), true );
                assignEntry( current_command.command, &receive_buffer[3], sizeof( current_command.command ), true);
                Serial.println( String( (int)current_command.command[0] ) );
                Serial.println( String( (int)current_command.command[1] ) );
            }
            else if( receive_buffer[1] == '\x30' ) //then it's a gps
            {
                Serial.println( "GPS received" );
                memset( &current_gtp.data[0], 0, sizeof( current_gtp.data ) );
                assignEntry( current_gtp.data, &receive_buffer[2], sizeof( current_gtp.data ), true );
                Serial.println( (char *)current_gtp.data );
            }            
        }
        
    }
    if( ( time_schedule + 1000 ) < millis() )
    {
        //assignment -- this is how we have to build data into our readings with the proper fill
        assignEntry( reading.time, String( millis() / 1000.0F ).c_str(), sizeof( reading.time ), false );
        assignEntry( reading.bank, "12", sizeof( reading.bank ), false );
        assignEntry( reading.so2_reading, "20.00", sizeof( reading.so2_reading ), false );
        assignEntry( reading.o3_reading, "15.00", sizeof( reading.o3_reading ), false );
        assignEntry( reading.no2_reading, "35.00", sizeof( reading.no2_reading ), false );
        assignEntry( reading.temp_reading, "25.25", sizeof( reading.temp_reading ), false );
        assignEntry( reading.pressure_reading, "1550.50", sizeof( reading.pressure_reading ), false );
        assignEntry( reading.humidity_reading, "50%", sizeof( reading.humidity_reading ), false );
        assignEntry( reading.pump_status, "ON", sizeof( reading.pump_status ), false );
        assignEntry( reading.peltier_status, "OFF", sizeof( reading.peltier_status ), false );
        assignEntry( reading.sd_status, "GOOD", sizeof( reading.sd_status ), false );
        assignEntry( reading.reading_status, " ACTIVE", sizeof( reading.reading_status ), false );
        //downlinking -- this is the logic to send data down to gondola (simple, works, beautiful)
        sendData( (byte *)&reading, sizeof( reading ) );
        time_schedule = millis();
    }

}


