#include <Servo.h>

//"#2", per my labeling on the board, is the wrist servo
//"#1", per my labeling on the board, is the pincher servo
#define SERVO_WRIST 1
#define SERVO_PINCH 2
Servo servo_pinch;
Servo servo_wrist;

int loop_counter, cycle;

void setup()
{
    pinMode( A14, INPUT ); //input from the pincher
    pinMode( A15, INPUT ); //input from the wrist

    servo_pinch.attach( SERVO_PINCH );
    servo_wrist.attach( SERVO_WRIST );
    for( int x = 1; x < 151; x++ )
    {
        int y = 0;
        //y = abs( 151 - x );
        servo_pinch.write( x );
        servo_wrist.write(y);
        delay( 10 );
    }

    loop_counter = 1;
    cycle = 0;
}

void loop()
{
    /* drive servos */
    servo_pinch.write( loop_counter );
    //servo_wrist.write( 150 - loop_counter );
 
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
