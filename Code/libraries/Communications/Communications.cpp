Communication::Communication( int baud_zero, int baud_one, int baud_two, int baud_three )
{
   ports[0] = new Serial( this, Serial.list()[0], baud_zero );
   ports[1] = new Serial( this, Serial.list()[1], baud_one );
   ports[2] = new Serial( this, Serial.list()[2], baud_two );
   ports[3] = new Serial( this, Serial.list()[3], baud_three );
}
