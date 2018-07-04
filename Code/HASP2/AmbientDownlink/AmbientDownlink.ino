#include <Wire.h>
#include <ram_funcs.h>
#include <ram_registers.h>
#include <hasp_arduino_sysclock.h>

/* Internal Time Register, all other registers will reference this */
unsigned long       internal_time_register  = 0;
ArduinoSysClock     sys_clock               ( internal_time_register );
ADA_output_register output_register;;
ADA_input_register  input_register;

void writeRegisters( int num_bytes )
{
    /* If we have a complete register written in there, read it */
    if( Wire.available() == sizeof( ADA_input_register ) )
    {
        /*
         * I don't have direct access to the buffer to do a memcpy, so I'll just use a byte pointer
         * Theoretically, this should be safe because the sizes of the register and the buffer are the "same" 
         */
        byte *input_ptr = (byte *)&input_register;
        for( int x = 0; x < sizeof( ADA_input_register ); x++ )
        {
            *input_ptr++ = Wire.read();
        }

        /*
         * Check the Data:
         *  - If its good, reset our fault flag
         *  - If its bad, set out fault flag
         */
        if( input_register.verifyCheckSums() )
        {
            output_register.write_fault = 0;
           //if fresh_packet is set, raise a downlink flag
        }
        else
        {
            output_register.write_fault = 1;
        }
    }
}

void readRegisters()
{
    Serial.println( "My time is: " + String( output_register.time_register ) );
    Wire.write( (byte *)&output_register, sizeof( ADA_output_register ) );
}

void setup()
{
    Serial.begin(9600);
    while( !Serial );

    Wire.begin( I2CADDRESS_ADA );
    Wire.onRequest( readRegisters );
    Wire.onReceive( writeRegisters );

    sys_clock.syncClock( 1234470131, millis() );
    

}

void updateTime()
{
    sys_clock.updateClock( millis() );
    output_register.time_register = internal_time_register;
}

void loop()
{
    //check for uplink
    //check for downlink flag
      // downlink and increase count so that rpi knows packet has gone out
    updateTime();
}
