/*
 * Spoof the GOAT
 */

#include "hasp_types.h"
#include "goat_funcs.h"

#define MAX_BUF 256    //A bit excessive, but better safe than sorry
SENSOR_READING reading;
GROUND_COMMAND current_command;
GTP_DATA current_gtp;

unsigned long long time_schedule;
byte receive_buffer[MAX_BUF];
unsigned int buffer_index;

void setup()
{
    Serial.begin( 1200 );
    while( !Serial );

    time_schedule = 0;
    buffer_index = 0;
}

void loop()
{
    if( Serial.available() )
    {
        switch( receiveData( Serial, receive_buffer, buffer_index, &reading, &current_command, &current_gtp ) )
        {
            case TRANS_INCOMPLETE: Serial.println( "Incomplete" ); break;
            case TRANS_COMMAND: Serial.println( "Command" ); break;
            case TRANS_DATA: Serial.println( "Data" ); break;
            case TRANS_GTP: Serial.println( "GTP" ); break;
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
        sendData( Serial, (byte *)&reading, sizeof( reading ) );
        time_schedule = millis();
    }

}


