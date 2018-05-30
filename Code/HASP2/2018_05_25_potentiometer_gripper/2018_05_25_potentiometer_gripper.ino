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
int potentiometer_readout; //10 to 800 or so
double pot_intermediate = 0;
int pot_intermediate_INT = 0;
int pot_target_INT = 0;
double pot_target = 0;
double wrist_position_translated = 0;
int wrist_position_translated_INT = 0;

void setup()
{
    pinMode( A14, INPUT ); //input from the pincher
    pinMode( A15, INPUT ); //input from the wrist
    pinMode( A12, INPUT ); //input from the potentiometer
    Serial.begin(9600);

    //servo_pinch.attach( SERVO_PINCH );
    servo_wrist.attach( SERVO_WRIST );
    wrist_pwm = WRIST_FULL_LEFT;
    pincher_pwm = PINCH_CLOSED;
    //servo_wrist.write(wrist_pwm);
    //servo_pinch.write(pincher_pwm);
}

void loop()
{
  pincher_position = analogRead( A14 );
  wrist_position = analogRead( A15 );
  potentiometer_readout = analogRead ( A12 );

  //***Convert potentiometer position to Batan target position (gets weird at bounds
  //since I don't have a good clean upper/lower max/min of the pot I'm using;
  //call it roughly 0 to 950ish
  //pot_target = (readout + readout_min) * (batan_max / readout_max)
  pot_target = double(potentiometer_readout) * ((double)150 / (double)950);
  pot_target_INT = (int)pot_target;

  //Zeno's PID
  wrist_position_translated = 1;


  //servo_pinch.write( pincher_pwm );
  //servo_wrist.write( pot_target );
  //Serial.print( "Pincher " + String( pincher_position ) );
  //Serial.print( " Wrist " + String( wrist_position ) );
  //Serial.println( " PWM " + String( wrist_pwm ) );
  Serial.print("ptargINT: " + String(pot_target_INT));
  Serial.println("\tPot readout: " + String(potentiometer_readout) + "\t Pot target: " + String(pot_target) + "\tWrist: " + String(wrist_position));
  delay( 50 );
}

