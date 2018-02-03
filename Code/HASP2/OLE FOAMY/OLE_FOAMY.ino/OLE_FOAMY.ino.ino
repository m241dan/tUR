#include <Servo.h>

#define S_PIN 9

Servo servo;

byte buf[256];
byte buf_index = 0;

void resetBuffer()
{
    memset( &buf[0], 0, 256 );
    buf_index = 0;

    pinMode( A0, INPUT );
}

void setup()
{
    servo.attach( S_PIN );
    servo.write( 1 );

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
    servo.write( value );
    Serial.println( "analogRead: " + String( analogRead( A0 ) ) );

    
}
