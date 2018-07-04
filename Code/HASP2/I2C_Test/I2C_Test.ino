#include <Wire.h>
#include <ram_funcs.h>

int counter = 0;
image_packet test;

unsigned long time_register = 0;
ArduinoSysClock sys_clock( time_register );

void setup() {
    Wire.begin(I2CADDRESS_AMBIENT);
    // put your setup code here, to run once:
    Serial.begin( 9600 );
    while ( !Serial );

    Wire.onRequest(i2cRequest);
    sys_clock.syncClock( 1234470131, millis() );
}

void loop() {
    // put your main code here, to run repeatedly:
    // Serial.println( "I'm a real boy!" );

    if( counter++ > 50 )
    {
        counter = 0;
        sys_clock.syncClock( 1234470131, millis() );
    }
    sys_clock.updateClock( millis() );
    Serial.println( "System Time is: " + String( time_register ) );
    delay( random( 500, 2500 ) );
    

}

void i2cRequest( )
{
    Serial.println( "Received a request for data" );
    test.imagepacket_position = random( 1, 99 );
    test.imagepacket_photo_number = counter++;
    test.imagepacket_meat[0] = 69;
    test.imagepacket_meat[50] = 96;
    Wire.write( (byte *)&test, sizeof( image_packet ) );
}

