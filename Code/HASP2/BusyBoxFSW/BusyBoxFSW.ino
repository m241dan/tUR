#include <Wire.h>
#include <ram_funcs.h>
#include <ram_registers.h>
#include <hasp_arduino_sysclock.h>
#include <ram_commands.h>
#include <SPI.h>
#include <SD.h>


BBOX_output_register    output_register;
BBOX_input_register     input_register;
ArduinoSysClock         sys_clock( output_register.time_register );
const int               BUTTON_BLU      = 24;           //blue button pin
const int               ROCKER_HORIZ    = 23;           //horizontal rocker switch (black)
const int               ROCKER_VERTI    = 22;           //vertical rocker switch (black)
const int               TOGGLE_HORIZ    = 0;            //horizontal toggle switch (metal)
const int               TOGGLE_VERTI    = 1;            //vertical toggle switch (metal)
const int               POTENTIOM_LEVER = A0;           //potentiometer (ANALOG pin, not digital)
const int               POTENTIOM_KNOB  = A1;           //potentiometer (ANALOG pin, not digital)
const float             POTENTIOM_TRUNC = 10.23;         //potentiometer truncator
const int               SD_CARD         = 4;            //"pin" for the SD card
void                 (*resetFunc)(void) = 0;            //calling this function will crash and reset the Arduino
int                     write_rate      = 1000;         // 0.20 Hz or 5 Seconds
uint32_t                last_write      = 0;
char log_name[15]                       = { 0 };

/* i2c write event (onReceive) */
void writeRegisters( int num_bytes )
{
    /* If we have a complete register written in there, read it */
    if( Wire.available() == sizeof( BBOX_input_register ) )
    {
        output_register.write_received = 1;
        output_register.write_fault = 0;
        output_register.command_fault = 0;
        /*
         * I don't have direct access to the buffer to do a memcpy, so I'll just use a byte pointer
         * Theoretically, this should be safe because the sizes of the register and the buffer are the "same" 
         */
        byte *input_ptr = (byte *)&input_register;
        for( int x = 0; x < sizeof( BBOX_input_register ); x++ )
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
            sys_clock.syncClock( input_register.sync_to, millis() );
            if( input_register.has_command )
            {
                switch( input_register.command_id )
                {
                    default:
                        output_register.command_fault = 1;
                        break;
                    case BBOX_WRITE_RATE[0]:
                        write_rate = (int)input_register.command_param * 1000;
                        break;
                     case BBOX_RESET[0]:
                        if( input_register.command_param == RESET_BYTE )
                        {
                            resetFunc();
                        }
                        else
                            output_register.command_fault = 1;
                        break;            
                }
            }
        }
        else
        {
            output_register.write_fault = 1;
        }
    }
    else
    {
        output_register.write_received = 0;
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
    Serial.begin( 9600 );

    pinMode(BUTTON_BLU,     INPUT_PULLUP);
    pinMode(ROCKER_HORIZ,   INPUT_PULLUP);
    pinMode(ROCKER_VERTI,   INPUT_PULLUP);
    pinMode(TOGGLE_HORIZ,   INPUT_PULLUP);
    pinMode(TOGGLE_VERTI,   INPUT_PULLUP);
    pinMode(POTENTIOM_LEVER,INPUT);
    pinMode(POTENTIOM_KNOB, INPUT);
  
    sys_clock.syncClock( 1234470131, millis() );
 }

void loop()
{
    output_register.rocker_horiz       = digitalRead( ROCKER_HORIZ     );
    output_register.rocker_verti       = digitalRead( ROCKER_VERTI     );
    output_register.toggle_horiz       = digitalRead( TOGGLE_HORIZ     );
    output_register.toggle_verti       = digitalRead( TOGGLE_VERTI     );
    output_register.button_blu         = digitalRead( BUTTON_BLU       )  != 1 ? output_register.time_register : output_register.button_blu;
    output_register.potentiometer_lever= (int)( analogRead( POTENTIOM_LEVER  ) / POTENTIOM_TRUNC );
    output_register.potentiometer_knob = (int)( analogRead( POTENTIOM_KNOB   ) / POTENTIOM_TRUNC );

    Serial.println( "Rocker Horiz: " + String( output_register.rocker_horiz ) );
    Serial.println( "Rocker Verti: " + String( output_register.rocker_verti ) );
    Serial.println( "Toggle Horiz: " + String( output_register.toggle_horiz ) );
    Serial.println( "Toggle Verti: " + String( output_register.toggle_verti ) );
    Serial.println( "Pot Level   : " + String( output_register.potentiometer_lever ) );
    Serial.println( "Pot Knob    : " + String( output_register.potentiometer_knob ) );
    Serial.println( "Blu Button  : " + String( output_register.button_blu ) );

    // call millis() each time for "most accurate" timing ;P
    sys_clock.updateClock( millis() );
    delay(1000);
}
