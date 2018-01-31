//Assign variable names to switch and LED pins for convenience
const int blueButton = 0;     //blue button pin
const int redButton = 1;      //red button pin
const int pushSwitch = 2;     //push switch pin
const int flickSwitch = 3;    //flick switch (one with metal rod) pin
const int potPin = A0;        //potentiometer (ANALOG pin 4, not digital)

const int LEDblue = 10;       //LED pin for blue button
const int LEDred = 11;        //LED pin for red button
const int LEDpush = 12;       //LED pin for push switch
const int LEDflick = 13;      //LED pin for flick switch

//Define variables for pin states, set to LOW
int blueState = 0;
int redState = 0;
int pushState = 0;
int flickState = 0;

void setup() {
  //initialize switches as inputs and LEDs as outputs
  pinMode(blueButton, INPUT);
  pinMode(redButton, INPUT);
  pinMode(pushSwitch, INPUT);
  pinMode(flickSwitch, INPUT);

  pinMode(LEDblue, OUTPUT);
  pinMode(LEDred, OUTPUT);
  pinMode(LEDpush, OUTPUT);
  pinMode(LEDflick, OUTPUT);

  //Initialize serial port for potentiometer
  Serial.begin(9600);
}

void loop() {
  //Read switch states
  blueState = digitalRead(blueButton);
  redState = digitalRead(redButton);
  pushState = digitalRead(pushSwitch);
  flickState = digitalRead(flickSwitch);

  //If switch is closed state is HIGH, turn on LED. If switch is open state is LOW, turn off LED.
  //Blue button
  if (blueState == HIGH) {
    digitalWrite(LEDblue, HIGH);
  }
  else {
    digitalWrite(LEDblue, LOW);
  }

  //Red button
  if (redState == HIGH) {
    digitalWrite(LEDred, HIGH);
  }
  else {
    digitalWrite(LEDred, LOW);
  }

 //Push switch
  if (pushState == HIGH) {
    digitalWrite(LEDpush, HIGH);
  }
  else {
    digitalWrite(LEDpush, LOW);
  }

  //Flick switch
  if (flickState == HIGH) {
    digitalWrite(LEDflick, HIGH);
   }
  else {
    digitalWrite(LEDflick, LOW);
  }

  //Read potentiometer and print values to serial port
  int potVal;
  potVal = analogRead(potPin);
  Serial.println(potVal);
}
