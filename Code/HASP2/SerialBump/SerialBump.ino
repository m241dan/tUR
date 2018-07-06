

void setup() {
  // put your setup code here, to run once:
    Serial1.begin( 9600 );
    while( !Serial );
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  Serial1.println( "Hi there!" );

}
