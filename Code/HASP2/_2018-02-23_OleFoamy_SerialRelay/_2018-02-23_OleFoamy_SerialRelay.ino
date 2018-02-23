//Dumbass fucking stupid sketch to replace our RS232 to USB reader.
//Reads in serial data coming out of vac chamber and passes it over USB to computer.
// 23 Feb 2018 - Jimmy :(

//#include "String.h"
String incomingString;

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);

}

void loop() {
  //MEGA-ONLY SYNTAX; change for UNO. -JEA
  static char buf[256];
  memset( &buf[0], 0, sizeof( 256 ) );
  incomingString = "";
  if( Serial1.available() ){
    for( int i = 0; i < 256; i++ )
    {
        buf[i]= Serial1.read();
        if( Serial1.available() == 0 )
            break;
    }
    incomingString = String( buf );
    Serial.print( incomingString );
  }
}
