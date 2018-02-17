#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define SERVO_6V 7
#define SERVO_12V 8
#define THERMO_1 50

Servo servo_6v;
Servo servo_12v;
OneWire oneWire( THERMO_1 );
DallasTemperature sensors ( &oneWire );
byte buf[STANDARD_BUF_SIZE];
byte buf_index = 0;

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

    Serial.begin( 9600 );
    while( !Serial );
    
    Serial.println( "Starting" );

    Serial1.begin( 9600 );
    while( !Serial );

    loop_counter = 1;
    cycle = 0;
}

void loop()
{
    /* drive servos */
    servo_6v.write( loop_counter );
    servo_12v.write( 150 - loop_counter );

   //sensors.requestTemperatures();

    /* | MILLIS | Servo6V Position | Servo12V Position | Servo6V Celsius | Servo12V Celsius | */
 /*   Serial.println( String( millis() ) + "," +
                    String( analogRead( A1 ) ) + "," +
                    String( analogRead( A2 ) ) + "," +
                    String( sensors.getTempCByIndex(0) ) + "," +
                    String( sensors.getTempCByIndex(1) ) ); */
  /*  Serial1.println( String( millis() ) + "," +
                    String( analogRead( A1 ) ) + "," +
                    String( analogRead( A2 ) ) + "," +
                    String( sensors.getTempCByIndex(0) ) + "," +
                    String( sensors.getTempCByIndex(1) ) ); */
 
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
