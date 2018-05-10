//Assign variable names to switch and LED pins for convenience
const int BUTTON_BLU    = 24;     //blue button pin
const int ROCKER_HORIZ  = 23;   //horizontal rocker switch (black)
const int ROCKER_VERTI  = 5;   //vertical rocker switch (black)
const int TOGGLE_HORIZ  = 22;   //horizontal toggle switch (metal)
const int TOGGLE_VERTI  = 6;   //vertical toggle switch (metal)
const int POTENTIOM_LEVER = A0;        //potentiometer (ANALOG pin, not digital)
const int POTENTIOM_KNOB = A1;         //potentiometer (ANALOG pin, not digital)

//Define variables for pin states, set to LOW
int state_button_blu    = 0;
int state_rocker_horiz  = 0;
int state_rocker_verti  = 0;
int state_toggle_horiz  = 0;
int state_toggle_verti  = 0;

void setup() {
  //initialize switches as inputs and LEDs as outputs
  pinMode(BUTTON_BLU, INPUT);
  pinMode(ROCKER_HORIZ, INPUT);
  pinMode(ROCKER_VERTI, INPUT);
  pinMode(TOGGLE_HORIZ, INPUT);
  pinMode(TOGGLE_VERTI, INPUT);
  pinMode(POTENTIOM_LEVER, INPUT);
  pinMode(POTENTIOM_KNOB, INPUT);

  //Initialize serial port for potentiometer
  Serial.begin(9600);
}

void loop() {
  //Read switch states
  state_button_blu    = digitalRead(BUTTON_BLU);
  state_rocker_horiz  = digitalRead(ROCKER_HORIZ);
  state_rocker_verti  = digitalRead(ROCKER_VERTI);
  state_toggle_horiz  = digitalRead(TOGGLE_HORIZ);
  state_toggle_verti  = digitalRead(TOGGLE_VERTI);


  Serial.print("BBox Status: ");
  Serial.print("Butan: \t");
  if (state_button_blu == HIGH) 
  {
    Serial.print("ON\t");
  }
  {
    Serial.print("--\t");
  }
  
  Serial.print("RockH: \t");
  if (state_rocker_horiz == HIGH) 
  {
    Serial.print("ON\t");
  }
  {
    Serial.print("--\t");
  }
  Serial.print("RockV: \t");
  if (state_rocker_verti == HIGH) 
  {
    Serial.print("ON\t");
  }
  {
    Serial.print("--\t");
  }
  Serial.print("ToggH: \t");
  if (state_toggle_horiz == HIGH) 
  {
    Serial.print("ON\t");
  }
  {
    Serial.print("--\t");
  }
  Serial.print("ToggV: \t");
  if (state_toggle_verti == HIGH) 
  {
    Serial.print("ON\t");
  }
  {
    Serial.print("--\t");
  }
  //Clean up the end of the line and wait a bit.
  Serial.println(" ");
  delay(50);
}
