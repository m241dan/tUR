#include <Servo.h>

#define S_PIN 7
#define S_PIN_2 8

Servo servo_one;
Servo servo_two;

byte buf[256];
byte buf_index = 0;

void resetBuffer()
{
    memset( &buf[0], 0, 256 );
    buf_index = 0;

    pinMode( A1, INPUT );
    pinMode( A2, INPUT );
}

void setup()
{
    servo_one.attach( S_PIN );
    servo_two.attach( S_PIN_2 );
    servo_one.write( map( 1, 1, 180, 1, 120 ) );
    servo_two.write( map( 1, 1, 180, 1, 120 ) );

    Serial.begin( 9600 );
    while( !Serial );

    resetBuffer();

    Serial.println( "Starting" );
}

void loop()
{
    static int value = 1;
    while( Serial.available() > 0 )
    {
        byte c = Serial.read();
        buf[buf_index++] = c;
        
        if( c != ';' )
            return;
        else
        {
            sscanf( buf, "%d;", &value );
            resetBuffer();
        }
    }
    servo_one.write( value );
    servo_two.write( value );
    Serial.println( "servo1: " + String( analogRead( A1 ) ) );
    Serial.println( "servo2: " + String( analogRead( A2 ) ) );    
}
