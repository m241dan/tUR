/*
 * Spoof the GOAT
 */

#include "hasp_types.h"

#define BUF_SIZE 4096    //A bit excessive, but better safe than sorry
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
    if( Serial.available() )
    {
        static byte receive_buffer[BUF_SIZE];
        byte c = Serial.read();
 
        if( c == '\x1' ) //we're at the beginning, scrap what we had and start over
        {
            memset( &receive_buffer[0], 0, BUF_SIZE );
            index = 0;
        }
        for( ; c != '\x0A' || Serial.available(); c = Serial.read(), index++ )
            receive_buffer[index] = c;
        receive_buffer[index] = c; // need to grab that last character incase it is not a '\x0A'
        
        if( c == '\x0A' ) //complete command received, parse it into the currents
        {
            if( receive_buffer[1] == '\x02' ) //then it's a command
            {
                assignEntry( current_command.checksum, receive_buffer[2], sizeof( current_command.checksum ) );
                assignEntry( current_command.command, receive_buffer[3], sizeof( current_command.command ) );
            }
            else if( receive_buffer[1] == '\x30' ) //then it's a gps
            {
                assignEntry( current_gtp.data, receive_buffer[2], sizeof( current_gtp.data ) );   
            }
        }   
    }
    if( ( time_schedule + 1000 ) < millis() )
    {
        //assignment -- this is how we have to build data into our readings with the proper fill
        assignEntry( reading.time, String( millis() / 100.0F ).c_str(), sizeof( reading.time ) );
        assignEntry( reading.bank, "12", sizeof( reading.bank ) );
        assignEntry( reading.so2_reading, "20.00", sizeof( reading.so2_reading ) );
        assignEntry( reading.o3_reading, "15.00", sizeof( reading.o3_reading ) );
        assignEntry( reading.no2_reading, "35.00", sizeof( reading.no2_reading ) );
        assignEntry( reading.temp_reading, "25.25", sizeof( reading.temp_reading ) );
        assignEntry( reading.pressure_reading, "1550.50", sizeof( reading.pressure_reading ) );
        assignEntry( reading.humidity_reading, "50%", sizeof( reading.humidity_reading ) );
        assignEntry( reading.pump_status, "ON", sizeof( reading.pump_status ) );
        assignEntry( reading.peltier_status, "OFF", sizeof( reading.peltier_status ) );
        assignEntry( reading.sd_status, "GOOD", sizeof( reading.sd_status ) );
        assignEntry( reading.reading_status, " ACTIVE", sizeof( reading.reading_status ) );
        //downlinking -- this is the logic to send data down to gondola (simple, works, beautiful)
        sendData( (byte *)&reading, sizeof( reading ) );
        time_schedule = millis();
    }

}


