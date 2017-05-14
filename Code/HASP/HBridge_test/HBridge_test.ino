/*
 * HASP HB: Simple H-Bridge Tester Circuit 
 * Author: Daniel R. Koris
 * Goal of Program: A proof of concept controller program for an H-Bridge controller a thermal
 *                  electric module (Peltier)
 * Required Components: Arduino, H-Bridge, Peltier
 */

#define pin_e   2
#define pin_one 3
#define pin_two 4
int mode = 0;

void setup()
{
    Serial.begin( 9600 );
    while( !Serial );

    pinMode( pin_e, OUTPUT );
    pinMode( pin_one, OUTPUT );
    pinMode( pin_two, OUTPUT );

    digitalWrite( pin_e, HIGH );
    digitalWrite( pin_one, HIGH );
    digitalWrite( pin_two, LOW );
}

void loop()
{
    if( Serial.available() )
    {
        char c;
        c = Serial.read();
        if( c == 's' )
        {
            if( digitalRead( pin_one ) == HIGH )
            {
                digitalWrite( pin_one, LOW );
                digitalWrite( pin_two, HIGH );
                mode = 1;
            }
            else
            {
                digitalWrite( pin_one, HIGH );
                digitalWrite( pin_two, LOW );
                mode = 0;
            }
        }
    }
    if( mode == 0 )
    {
        Serial.println( "Mode is 0" );
    }
    else
    {
        Serial.println( "Mode is 1" );
    }
    delay( 1000 );

}
