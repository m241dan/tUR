//Assign variable names to switch and LED pins for convenience
const int BUTTON_BLU    = 24;     //blue button pin
const int ROCKER_HORIZ  = 23;   //horizontal rocker switch (black)
const int ROCKER_VERTI  = 22;   //vertical rocker switch (black)
const int TOGGLE_HORIZ  = 5;   //horizontal toggle switch (metal)
const int TOGGLE_VERTI  = 6;   //vertical toggle switch (metal)
const int POTENTIOM_LEVER = A0;        //potentiometer (ANALOG pin, not digital)
const int POTENTIOM_KNOB = A1;         //potentiometer (ANALOG pin, not digital)

//Define variables for pin states, set to LOW
int state_button_blu    = 0;
int state_rocker_horiz  = 0;
int state_rocker_verti  = 0;
int state_toggle_horiz  = 0;
int state_toggle_verti  = 0;
int state_pot_lever     = 0;
float potentiometer_truncator_hack = 10.23;

void setup() 
{
  //initialize switches as inputs
  pinMode(BUTTON_BLU,     INPUT_PULLUP);
  pinMode(ROCKER_HORIZ,   INPUT_PULLUP);
  pinMode(ROCKER_VERTI,   INPUT_PULLUP);
  pinMode(TOGGLE_HORIZ,   INPUT_PULLUP);
  pinMode(TOGGLE_VERTI,   INPUT_PULLUP);
  pinMode(POTENTIOM_LEVER,INPUT_PULLUP);
  pinMode(POTENTIOM_KNOB, INPUT_PULLUP);

  //Initialize serial port for potentiometer
  Serial.begin(9600);
}

void loop() 
{
  //Read switch states
  state_button_blu    = digitalRead(BUTTON_BLU);
  state_rocker_horiz  = digitalRead(ROCKER_HORIZ);
  state_rocker_verti  = digitalRead(ROCKER_VERTI);
  state_toggle_horiz  = digitalRead(TOGGLE_HORIZ);
  state_toggle_verti  = digitalRead(TOGGLE_VERTI);
  state_pot_lever     = analogRead(POTENTIOM_LEVER);

  Serial.print("BBox ");
  Serial.print("Btn: \t");
  if (state_button_blu == HIGH) 
  {
    Serial.print("ON\t");
  }
  else
  {
    Serial.print("--\t");
  }
  
  Serial.print("RockH: \t");
  if (state_rocker_horiz == HIGH) 
  {
    Serial.print("ON\t");
  }
  else
  {
    Serial.print("--\t");
  }
  Serial.print("RockV: \t");
  if (state_rocker_verti == HIGH) 
  {
    Serial.print("ON\t");
  }
  else
  {
    Serial.print("--\t");
  }
  Serial.print("ToggH: \t");
  if (state_toggle_horiz == HIGH) 
  {
    Serial.print("ON\t");
  }
  else
  {
    Serial.print("--\t");
  }
  Serial.print("ToggV: \t");
  if (state_toggle_verti == HIGH) 
  {
    Serial.print("ON\t");
  }
  else
  {
    Serial.print("--\t");
  }
  /*Hack the 0-1023 range of the potentiometer down to 0-100 to reduce data 
  and clean out noise. 100% my idea. I'm so smart. -JA*/
  state_pot_lever = state_pot_lever / potentiometer_truncator_hack;
  Serial.print("Pot: ");
  Serial.print((int)state_pot_lever);
  
  /*Clean up the end of the line and wait a bit.*/
  Serial.println("");
  delay(500);
}
