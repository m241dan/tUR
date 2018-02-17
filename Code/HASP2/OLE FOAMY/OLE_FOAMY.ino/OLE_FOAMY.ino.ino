#include <Servo.h>

#define SERVO_6V 7
#define SERVO_12V 8

Servo servo_6v;
Servo servo_12v;


int loop_counter, cycle;

void setup()
{
    pinMode( A1, INPUT );
    pinMode( A2, INPUT );

    servo_6v.attach( SERVO_6V );
    servo_6v.write( 1 );
    servo_12v.attach( SERVO_12V );
    for( int x = 1; x < 151; x++ )
    {
        servo_12v.write( x );
        delay( 10 );
    }

    loop_counter = 1;
    cycle = 0;
}

void loop()
{
    /* drive servos */
    servo_6v.write( loop_counter );
    servo_12v.write( 150 - loop_counter );
 
    /* loop handling and delay */
    if( cycle == 0 )
    {
        loop_counter++;
        if( loop_counter > 150 )
            cycle = 1;
    }
    else
    {
        loop_counter--;
        if( loop_counter < 2 )
            cycle = 0;
    }
    delay( 100 );
}
