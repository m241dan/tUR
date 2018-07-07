#include <Wire.h>
#include <ram_funcs.h>
#include <ram_registers.h>
#include <hasp_arduino_sysclock.h>
#include <ram_commands.h>


BBOX_output_register    output_register;
BBOX_input_register     input_register;
ArduinoSysClock         sys_clock( output_register.time_register );
const int               BUTTON_BLU      = 24;           //blue button pin
const int               ROCKER_HORIZ    = 23;           //horizontal rocker switch (black)
const int               ROCKER_VERTI    = 22;           //vertical rocker switch (black)
const int               TOGGLE_HORIZ    = 5;            //horizontal toggle switch (metal)
const int               TOGGLE_VERTI    = 6;            //vertical toggle switch (metal)
const int               POTENTIOM_LEVER = A0;           //potentiometer (ANALOG pin, not digital)
const int               POTENTIOM_KNOB  = A1;           //potentiometer (ANALOG pin, not digital)
const float             POTENTIOM_TRUNC = 1023;
void                 (*resetFunc)(void) = 0;

/* i2c write event (onReceive) */
void writeRegisters( int num_bytes )
{
    /* If we have a complete register written in there, read it */
    if( Wire.available() == sizeof( BBOX_input_register ) )
    {
        /*
         * I don't have direct access to the buffer to do a memcpy, so I'll just use a byte pointer
         * Theoretically, this should be safe because the sizes of the register and the buffer are the "same" 
         */
        byte *input_ptr = (byte *)&input_register;
        for( int x = 0; x < sizeof( BBOX_input_register ); x++ )
        {
            *input_ptr++ = Wire.read();
        }
        output_register.writes_received++;
        /*
         * Check the Data:
         *  - If its good, reset our fault flag
         *  - If its bad, set out fault flag
         */
        if( input_register.verifyCheckSums() )
        {
            output_register.write_fault = 0;
            sys_clock.syncClock( input_register.sync_to, millis() );
            if( input_register.has_command )
            {
             //   switch( input_register.command_id )
             //  {
              //      case 
              //  }
            }
        }
        else
        {
            output_register.write_fault = 1;
        }
    }
    Serial.println( "Test" );
}

/* i2c read event (onRequest) */
void readRegisters()
{
    output_register.setCheckSums();
    Wire.write( (byte *)&output_register, sizeof( BBOX_output_register ) );
}

void setup()
{
    Wire.begin( I2CADDRESS_BBOX );
    Wire.onRequest( readRegisters );
    Wire.onReceive( writeRegisters );

    pinMode(BUTTON_BLU,     INPUT_PULLUP);
    pinMode(ROCKER_HORIZ,   INPUT_PULLUP);
    pinMode(ROCKER_VERTI,   INPUT_PULLUP);
    pinMode(TOGGLE_HORIZ,   INPUT_PULLUP);
    pinMode(TOGGLE_VERTI,   INPUT_PULLUP);
    pinMode(POTENTIOM_LEVER,INPUT_PULLUP);
    pinMode(POTENTIOM_KNOB, INPUT_PULLUP);
  
    sys_clock.syncClock( 1234470131, millis() );
}

void loop()
{
    output_register.rocker_horiz       = digitalRead( ROCKER_HORIZ     );
    output_register.rocker_verti       = digitalRead( ROCKER_VERTI     );
    output_register.toggle_horiz       = digitalRead( TOGGLE_HORIZ     );
    output_register.toggle_verti       = digitalRead( TOGGLE_VERTI     );
    output_register.button_blu         = digitalRead( BUTTON_BLU       );
    output_register.potentiometer_lever= digitalRead( POTENTIOM_LEVER  );
    output_register.potentiometer_knob = digitalRead( POTENTIOM_KNOB   );
    sys_clock.updateClock( millis() );
}
