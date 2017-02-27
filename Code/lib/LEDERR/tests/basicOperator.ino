#include <LEDERR.h>

//Initialize the library class
LEDERR led;

//Array to hold the pins, this allows multiple pin manipulation.
//You can change this value anytime, this assumes three Leds
//Are connected to pins 4, 3, and 2 of an Arduino Uno/Mega
byte  ledPins[] = {4, 3, 2};

void setup()
{
   //Setup the pins
   led.setUpLed(ledPins);

   //Open the serial port at 9600
   Serial.begin(9600);
}

void loop() {
   //Example showing the use of the LEDERR library
   while (Serial.available() > 0)
   {
      //Read the incoming bytes
      byte ledStatus = Serial.read();

      switch (ledStatus)
      {
         case '0':
            Serial.println("Turning the Led/Leds off");

            //Turn off the leds
            led.turnOff(ledPins);
            break;
      
         case '1':
            Serial.println("Turning the Led/Leds on");

            //Turn On the Leds
            led.turnOn(ledPins);
            break;

         case '2':
            Serial.println("Flashing the Led/Leds");

            //Flas the LEDs for error purposes
            led.flashLed(ledPins, 1000); //This will flash for 500ms
            break;
      }
   }
}
