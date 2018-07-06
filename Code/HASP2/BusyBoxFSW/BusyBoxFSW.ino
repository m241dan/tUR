#include <Wire.h>
#include <ram_funcs.h>
#include <ram_registers.h>
#include <hasp_arduino_sysclock.h>

BB_output_register  output_register;
BB_input_register   input_register;
ArduinoSysClock     sys_clock( output_register.time_register );

/* i2c write event (onReceive) */
void writeRegisters( int num_bytes )
{
    /* If we have a complete register written in there, read it */
    if( Wire.available() == sizeof( BB_input_register ) )
    {
        /*
         * I don't have direct access to the buffer to do a memcpy, so I'll just use a byte pointer
         * Theoretically, this should be safe because the sizes of the register and the buffer are the "same" 
         */
        byte *input_ptr = (byte *)&input_register;
        for( int x = 0; x < sizeof( BB_input_register ); x++ )
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
            sys_clock.syncClock( input_register.sync_to, millis() );    
        }
        else
        {
            output_register.write_fault = 1;
        }
    }
}

/* i2c read event (onRequest) */
void readRegisters()
{
    output_register.setCheckSums();
    Wire.write( (byte *)&output_register, sizeof( BB_output_register ) );
}

void setup()
{
    Wire.begin( I2CADDRESS_BBOX );
    Wire.onRequest( readRegisters );
    Wire.onReceive( writeRegisters );

    sys_clock.syncClock( 1234470131, millis() );
}

void loop()
{
    sys_clock.updateClock( millis() );
}
