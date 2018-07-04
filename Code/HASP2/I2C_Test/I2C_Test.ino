
#include <Wire.h>
#include <ram_funcs.h>



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
  Serial.println( "Received a request for bytes" );
  Wire.write( random( 5, 15 ) );
}

