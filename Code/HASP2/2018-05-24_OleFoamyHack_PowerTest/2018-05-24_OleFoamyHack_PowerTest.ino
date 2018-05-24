#include <Servo.h>
#include "String.h"

//"#1", per my labeling on the board, is the pincher servo. -JA
//"#2", per my labeling on the board, is the wrist servo. -JA
#define SERVO_PINCH 3
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
#define ROTATE_RIGHT 0
#define ROTATE_LEFT 1
#define PINCH_OPENING 0
#define PINCH_CLOSING 1

#define PINCH_OPEN 1
#define PINCH_CLOSED 62

#define WRIST_FULL_LEFT 150
#define WRIST_FULL_RIGHT 1

Servo servo_pinch;
Servo servo_wrist;

int wrist_pwm, pincher_pwm;
int wrist_mode, pincher_mode;
int wrist_position, pincher_position;

void setup()
{
    pinMode( A14, INPUT ); //input from the pincher
    pinMode( A15, INPUT ); //input from the wrist
    Serial.begin(9600);

    servo_pinch.attach( SERVO_PINCH );
    servo_wrist.attach( SERVO_WRIST );
    wrist_pwm = WRIST_FULL_LEFT;
    pincher_pwm = PINCH_CLOSED;
    servo_wrist.write(wrist_pwm);
    servo_pinch.write(pincher_pwm);
    wrist_mode = ROTATE_RIGHT;
    pincher_mode = PINCH_OPENING;    
}

void loop()
{
  pincher_position = analogRead( A14 );
  wrist_position = analogRead( A15 );

  if( wrist_mode == ROTATE_RIGHT )
  {
    wrist_pwm--;
    if( wrist_pwm == WRIST_FULL_RIGHT )
    {
      wrist_mode = ROTATE_LEFT;
    }
  }
  else
  {
    wrist_pwm++;
    if( wrist_pwm == WRIST_FULL_LEFT )
    {
      wrist_mode = ROTATE_RIGHT;
    }
  }
  
  if( pincher_mode == PINCH_OPENING )
  {
    pincher_pwm--;
    if( pincher_pwm == PINCH_OPEN )
    {
      pincher_mode = PINCH_CLOSING;
    }
  }
  else
  {
    pincher_pwm++;
    if( pincher_pwm == PINCH_CLOSED )
    {
      pincher_mode = PINCH_OPENING;
    }
  }

  servo_pinch.write( pincher_pwm );
  servo_wrist.write( wrist_pwm );
  Serial.print( "Pincher " + String( pincher_position ) );
  Serial.print( " Wrist " + String( wrist_position ) );
  Serial.println( " PWM " + String( wrist_pwm ) );
  delay( 100 );
}
//void loop()
//{
//    /* drive servos */
//    servo_pinch.write( loop_pinch );
//    target_wrist = loop_wrist;
//    servo_wrist.write( target_wrist );
//    
//    /* read Batan servo position */
//    int pinch_readout = analogRead(A14);
//    int wrist_readout = analogRead(A15);
// 
//    /* WRIST loop handling and delay 
//    if we are turning CW, keep turning CW until we reach max CW, then start turning CCW  
//    if we are turning CCW, keep turning CCW until we reach max CCW, then start turning CW
//    */  
//    if( cycle_wrist == 0 )
//    {
//        /* cycle_wrist == 0 means turning CW, == 1 means turning CCW*/
//      if ( loop_wrist < WRIST_MAX_CCW_BATAN)
//      {
//        loop_wrist++;  
//      }
//      if ( loop_wrist >= WRIST_MAX_CCW_BATAN )
//      {
//        cycle_wrist = 1;
//      }
//      if ( loop_wrist <= WRIST_MIN_CCW_BATAN)
//      {
//        //Logically shouldn't happen, but accounting for the edge case. -JA
//        cycle_wrist = 1;
//      }
//    }
//    if ( cycle_wrist == 1)
//    {
//      if ( loop_wrist > WRIST_MIN_CCW_BATAN)
//      {
//        loop_wrist--;  
//      }
//      if ( loop_wrist <= WRIST_MIN_CCW_BATAN)
//      {
//        cycle_wrist = 0;
//        if (loop_wrist <= (WRIST_MIN_CCW_BATAN - 10))
//        {
//          loop_wrist = WRIST_MIN_CCW_BATAN;
//        }
//      }
//    }  
//
///* PINCH loop handling and delay 
//    if we are opening, keep opening until we are all the way open then immediately start shutting
//    if we are shutting, keep shutting until we are all the way shut
//    if we are shut, wait 3 seconds then start opening
//    */  
//    if( cycle_pinch == 0 )
//    {
//        /* cycle_pinch == 0 means opening, == 1 means shutting*/
//      if ( loop_pinch >= PINCH_SHUT_BATAN)
//      {
//        loop_pinch++;  
//      }
//      if ( loop_pinch < PINCH_OPEN_BATAN )
//      {
//        cycle_pinch = 1;
//      }
//    }
//    if ( cycle_pinch == 1)
//    {
//      if ( loop_pinch <= PINCH_OPEN_BATAN)
//      {
//        loop_pinch--;  
//      }
//      if ( loop_pinch > PINCH_SHUT_BATAN)
//      {
//        cycle_pinch = 0;
//      }
//    }
//    
//    delay( 100 );
//    Serial.println( "[" + String(cycle_wrist) + "]" + " LoopWrist: \t" + String(loop_wrist) + "\t" 
//                    "[" + String(cycle_pinch) + "]" + " Pinch loop: " + String(loop_pinch) +
//                    "\t PinchPos: " + String(pinch_readout) + 
//                    "\t WristPos: " + String(wrist_readout));
//}
