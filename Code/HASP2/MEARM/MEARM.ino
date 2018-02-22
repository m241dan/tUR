#include <Servo.h>
//THINGS IN CAPS ARE FOR THE PORT TO ROBOPILIB

//CONNECT TO ROBOPI
//#RoboPi.RoboPiInit(“/dev/ttyAMA0”,115200)

//SET PIN 11 TO OUTPUT
//RoboPi.pinMode(11,RoboPi.PWM)
//DO THE SAME FOR ALL OF THEM

#define BASE_PIN 11
#define SERVO_2_PIN 10
#define SERVO_3_PIN 9
#define SERVO_4_PIN 999//:TODO: CHANGE
#define SERVO_5_PIN 888//CHANGE
#define SERVO_6_PIN 777//CHANGE
#define CLAW_PIN 6

//7 servos, instead of 4
//In order from the base to the claw
Servo base; //Servo_1
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_5;
Servo servo_6;
Servo claw; //Servo_7

//1 - 120 range of motion

//TODO: Possibly 2 arrays?
//Could make loopng through a whole lot easier
//Maybe later...
int base_cur_pos = 1;//TODO: MAKE USE IF servoRead(int pin) FOR ROBOPI, SO WE HAVE THE ACTUAL VALUE AND NOT A VALUE WE THOUGHT WE ENTERED.
int servo_2_cur_pos = 1;
int servo_3_cur_pos = 1;
int servo_4_cur_pos = 1;
int servo_5_cur_pos = 1;
int servo_6_cur_pos = 1;
int claw_cur_pos = 1;

int base_final_pos = 1;
int servo_2_final_pos = 1;
int servo_3_final_pos = 1;
int servo_4_final_pos = 1;
int servo_5_final_pos = 1;
int servo_6_final_pos = 1;
int claw_final_pos = 1;

//To be used to adjust the turning speed
int turnDelay;

byte buf[256];
byte buf_index = 0;

void resetBuffer()
{
    memset( &buf[0], 0, 256 );
    buf_index = 0;
}

void setup()
{
  //Attach the pins to each servo
    base.attach(BASE_PIN);
    servo_2.attach(SERVO_2_PIN);
    servo_3.attach(SERVO_3_PIN);
    servo_4.attach(SERVO_4_PIN);
    servo_5.attach(SERVO_5_PIN);
    servo_6.attach(SERVO_6_PIN);
    claw.attach(CLAW_PIN);

    //TODO: Adjust these startng values
    base.write( 90 );
    servo_2.write( 2 );
    servo_3.write( 70 );
    servo_4.write( 20 );
    servo_5.write( 20 );
    servo_6.write( 20 );
    claw.write( 120 );

    turnDelay = 15;

    Serial.begin( 9600 );
    while( !Serial );

    resetBuffer();

    Serial.println( "Starting" );
}

void moveServo(Servo s, int &cur, int &final) {
    if( final != cur ) {
      //TODO: Smooth the transition from one place to the next
        while(final != cur){
          if(cur > final) {
            s.write(cur--);//analogWrite(int pin, int val)
            delay( turnDelay );
        
          }
          else{
            s.write(cur++);
            delay(turnDelay);

          }
        }

        //cur = final;
    }
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
              //B = BASE 
              //2 = SERV0 2
              //3 = SERVO 3...
              // ...
              //C = CLAW
              
                default:
                    Serial.println( "Bad command." );
                    break;
                case 'b':
                    base_final_pos = value;
                    Serial.print( "Setting Base to " );
                    Serial.println( base_final_pos );                    
                    break;
                case '2':
                    servo_2_final_pos = value;
                    Serial.print( "Setting Servo 2 to " );
                    Serial.println( servo_2_final_pos );
                    break;
                case '3':
                    servo_3_final_pos = value;
                    Serial.print( "Setting Servo 3 to " );
                    Serial.println( servo_3_final_pos );                
                    break;
                case '4':
                    servo_4_final_pos = value;
                    Serial.print( "Setting Servo 4 to " );
                    Serial.println( servo_4_final_pos );                
                    break;
                case '5':
                    servo_5_final_pos = value;
                    Serial.print( "Setting Servo 5 to " );
                    Serial.println( servo_5_final_pos );                
                    break;
                case '6':
                    servo_6_final_pos = value;
                    Serial.print( "Setting Servo 6 to " );
                    Serial.println( servo_6_final_pos );                
                    break;
                case 'c':
                    claw_final_pos = value;
                    Serial.print( "Setting C to " );
                    Serial.println( claw_final_pos );                
                    break;
                case 'd':
                    turnDelay = value;
                    Serial.print( "Setting Delay to " );
                    Serial.println( turnDelay );                
                    break;
            }
            resetBuffer();
        }
    }
    //Move the servon to desired location
    moveServo(base, base_cur_pos, base_final_pos);
    moveServo(servo_2, servo_2_cur_pos, servo_2_final_pos);
    moveServo(servo_3, servo_3_cur_pos, servo_3_final_pos);
    moveServo(servo_4, servo_4_cur_pos, servo_4_final_pos);
    moveServo(servo_5, servo_5_cur_pos, servo_5_final_pos);
    moveServo(servo_6, servo_6_cur_pos, servo_6_final_pos);
    moveServo(claw, claw_cur_pos, claw_final_pos);


//    if( x_final_pos != x_cur_pos )
   // {
     //   x_cur_pos = x_final_pos;
    //    x_servo.write( x_cur_pos );
   // }

}



