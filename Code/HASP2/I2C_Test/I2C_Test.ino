
#include <Wire.h>
#include <ram_funcs.h>

image_packet test;

void setup() {  
  Wire.begin(I2CADDRESS_AMBIENT);
  // put your setup code here, to run once:
  Serial.begin( 9600 );
  while( !Serial );

  Wire.onRequest(i2cRequest);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println( "I'm a real boy!" );
  delay( 100 );

}

void i2cRequest( )
{
  Serial.println( "Received a request for data" );
  test.imagepacket_position = random( 1, 5 );
  test.imagepacket_photo_number = 10;
  test.imagepacket_meat[0] = 69;
  test.imagepacket_meat[50] = 96;
  Wire.write( (byte *)&test, sizeof( image_packet ) );
}

