#define SAMPLE_RATE 1000
int time_schedule;
int iteration;

void setup()
{
    Serial.begin( 9600 );
    while( !Serial );
    iteration = 0;
    time_schedule = 0;
}

void loop()
{
    if( ( time_schedule + SAMPLE_RATE ) < millis() )
    {
        String buf = "";

        time_schedule = millis();
        iteration++;
        
        buf += String( iteration ) + " ";
        buf += String( 1000 ) + " ";
        buf += String( 24 ) + " ";
        buf += String( 10 ) + " ";
        buf += String( 1.5 ) + " " + String( 1.5 ) + " " + String( 1.5 ) + " ";
        buf += String( 1.5 ) + " " + String( 1.5 ) + " " + String( 1.5 ) + " ";
        buf += String( 1.5 ) + " " + String( 1.5 ) + " " + String( 1.5 ) + " ";

        Serial.println( buf );
    }
}
