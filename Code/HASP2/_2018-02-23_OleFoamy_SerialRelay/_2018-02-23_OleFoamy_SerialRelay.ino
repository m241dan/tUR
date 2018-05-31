//Dumbass fucking stupid sketch to replace our RS232 to USB reader.
//Reads in serial data coming out of vac chamber and passes it over USB to computer.
// 25 May 2018 changed so both ends of this are on UNOs. -JA

//#include "String.h"
String incomingString;

void setup() {
  //Serial1.begin(9600); //Mega syntax
  Serial.begin(9600);

}

void loop() 
{
  static char buf[256];
  memset( &buf[0], 0, sizeof( 256 ) );
  incomingString = "";
  if( Serial.available() )
  {
    for( int i = 0; i < 256; i++ )
    {
        buf[i]= Serial.read();
        if( Serial.available() == 0 )
            break;
    }  
    incomingString = String( buf );
    Serial.print( incomingString );
  }
}
