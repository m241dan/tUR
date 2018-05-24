#include <Servo.h>
#include "String.h"

//"#1", per my labeling on the board, is the pincher servo. -JA
//"#2", per my labeling on the board, is the wrist servo. -JA
#define SERVO_PINCH 1
#define SERVO_WRIST 2
/*~~~
     *  30 from analog wrist == 0 physical, maxed out clockwise (looking along gripper from behind)
     * 500 from analog wrist == 150 physical, maxed out CCW
     * 275 from analog pinch == gripper closed
     *  45 from analog pinch == gripper open ~~~*/
#define WRIST_MIN_CCW_BATAN 0
#define WRIST_MAX_CCW_BATAN 150
#define WRIST_MIN_CCW_ANALOG 30
#define WRIST_MAX_CCW_ANALOG 500
#define PINCH_SHUT_ANALOG    275
#define PINCH_SHUT_BATAN     0
#define PINCH_OPEN_ANALOG    45
#define PINCH_OPEN_BATAN     150

Servo servo_pinch;
Servo servo_wrist;

int loop_pinch, loop_wrist, cycle_wrist, cycle_pinch, target_wrist, target_pinch;

void setup()
{
    pinMode( A14, INPUT ); //input from the pincher
    pinMode( A15, INPUT ); //input from the wrist
    Serial.begin(9600);

    //servo_pinch.attach( SERVO_PINCH );
    servo_wrist.attach( SERVO_WRIST );
    servo_wrist.write(50);
    
    /*for( int x = 1; x < 151; x++ )
    {
        int y = 0;
        //y = abs( 151 - x );
        //servo_pinch.write( x );
        servo_wrist.write(y);
        delay( 10 );
    }*/

    loop_wrist = 50;
    loop_pinch = 1;
    cycle_wrist = 0;
    cycle_pinch = 0;
}

void loop()
{
    /* drive servos */
    //servo_pinch.write( loop_pinch );
    target_wrist = loop_wrist;
    servo_wrist.write( target_wrist );
    
    /* read Batan servo position */
    int pinch_readout = analogRead(A14);
    int wrist_readout = analogRead(A15);
 
    /* WRIST loop handling and delay 
    if we are turning CW, keep turning CW until we reach max CW, then start turning CCW  
    if we are turning CCW, keep turning CCW until we reach max CCW, then start turning CW
    */  
    if( cycle_wrist == 0 )
    {
        /* cycle_wrist == 0 means turning CW, == 1 means turning CCW*/
      if ( loop_wrist < WRIST_MAX_CCW_BATAN)
      {
        loop_wrist++;  
      }
      if ( loop_wrist >= WRIST_MAX_CCW_BATAN )
      {
        cycle_wrist = 1;
      }
      if ( loop_wrist <= WRIST_MIN_CCW_BATAN)
      {
        //Logically shouldn't happen, but accounting for the edge case. -JA
        cycle_wrist = 1;
      }
    }
    if ( cycle_wrist == 1)
    {
      if ( loop_wrist > WRIST_MIN_CCW_BATAN)
      {
        loop_wrist--;  
      }
      if ( loop_wrist <= WRIST_MIN_CCW_BATAN)
      {
        cycle_wrist = 0;
        if (loop_wrist <= (WRIST_MIN_CCW_BATAN - 10))
        {
          loop_wrist = WRIST_MIN_CCW_BATAN;
        }
      }
    }  
    
    delay( 100 );
    Serial.println( "LoopWrist: \t" + String(loop_wrist) + "\t " + String(cycle_wrist) +
                    "\t Pinch loop: " + String(loop_pinch) +
                    "\t PinchPos: " + String(pinch_readout) + 
                    "\t WristPos: " + String(wrist_readout));
}
