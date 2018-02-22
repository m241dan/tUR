#include <Servo.h>

#define X_PIN 11
#define Y_PIN 10
#define Z_PIN 9
#define CLAW_PIN 6

Servo x_servo;
Servo y_servo;
Servo z_servo;
Servo c_servo;

int x_cur_pos = 1;
int y_cur_pos = 1;
int z_cur_pos = 1;
int c_cur_pos = 1;

int x_final_pos = 1;
int y_final_pos = 1;
int z_final_pos = 1;
int c_final_pos = 1;

int rate = 0;

byte buf[256];
byte buf_index = 0;

void resetBuffer()
{
    memset( &buf[0], 0, 256 );
    buf_index = 0;
}

void setup()
{
    x_servo.attach( X_PIN );
    y_servo.attach( Y_PIN );
    z_servo.attach( Z_PIN );
    c_servo.attach( CLAW_PIN );

    x_servo.write( 1 );
    y_servo.write( 1 );
    z_servo.write( 1 );
    c_servo.write( 1 );

    Serial.begin( 9600 );
    while( !Serial );

    resetBuffer();

    Serial.println( "Starting" );
}

void loop()
{
    while( Serial.available() > 0 )
    {
        byte c = Serial.read();
        buf[buf_index++] = c;
        
        if( c != ';' )
            return;
        else
        {
            char desired = 0;
            int value = 0;
            sscanf( buf, "%c=%d;", &desired, &value );
            switch( desired )
            {
                default:
                    Serial.println( "Bad command." );
                    break;
                case 'x':
                    x_final_pos = value;
                    Serial.print( "Setting X to " );
                    Serial.println( x_final_pos );                    
                    break;
                case 'y':
                    y_final_pos = value;
                    Serial.print( "Setting Y to " );
                    Serial.println( y_final_pos );
                    break;
                case 'z':
                    z_final_pos = value;
                    Serial.print( "Setting Z to " );
                    Serial.println( z_final_pos );                
                    break;
                case 'c':
                    c_final_pos = value;
                    Serial.print( "Setting C to " );
                    Serial.println( c_final_pos );                
                    break;
                case 'r':
                    rate = value;
                    break;
            }
            resetBuffer();
        }
    }

    if( x_final_pos != x_cur_pos )
    {
        x_cur_pos = x_final_pos;
        x_servo.write( x_cur_pos );
    }
    if( y_final_pos != y_cur_pos )
    {
        y_cur_pos = y_final_pos;
        y_servo.write( y_cur_pos );
    }
  //  if( z_final_pos != z_cur_pos )
   // {
     //   z_cur_pos = z_final_pos;
        z_servo.write( z_cur_pos );
   // }
    if( c_final_pos != c_cur_pos )
    {
        c_cur_pos = c_final_pos;
        c_servo.write( c_cur_pos );
    }
}
