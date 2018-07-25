/*  Sketch to run the 2018 HASP RAM BusyBox.
 *  Listens to its various components (buttons, switches, 
 *  potentiometers, and maybe eventually Hall sensors) 
 *  then reports status changes back to the Master.
 *  
 *  June 2018 -DK & JA
 */
#include "ram_funcs.h";
#include <Wire.h>

byte x = 0;
const int BUTTON_BLU      = 24;     //blue button pin
const int ROCKER_HORIZ    = 23;   //horizontal rocker switch (black)
const int ROCKER_VERTI    = 22;   //vertical rocker switch (black)
const int TOGGLE_HORIZ    = 5;   //horizontal toggle switch (metal)
const int TOGGLE_VERTI    = 6;   //vertical toggle switch (metal)
const int POTENTIOM_LEVER = A0;        //potentiometer (ANALOG pin, not digital)
const int POTENTIOM_KNOB  = A5;         //potentiometer (ANALOG pin, not digital)

//Define variables for pin states, set to LOW
int present_bbox_rocker_horiz    = 0;
int present_bbox_rocker_verti    = 0;
int present_bbox_toggle_horiz    = 0;
int present_bbox_toggle_verti    = 0;
int present_bbox_button_blu      = 0;
int present_bbox_button_blu_press_recorded    = 0;
int present_bbox_flap            = 0;
int present_bbox_potentiometer_lever = 0;
int present_bbox_potentiometer_knob = 0;
float potentiometer_truncator_hack = 10.23; //Thus defined because the default range of this pot is 0-1023

//Hacking shit back in for standalone busynox function
int bbox_rocker_horiz    = 0;
int bbox_rocker_verti    = 0;
int bbox_toggle_horiz    = 0;
int bbox_toggle_verti    = 0;
int bbox_button_blu      = 0;
int bbox_button_blu_press_recorded    = 0;
int bbox_flap            = 0;
int bbox_potentiometer_lever = 0;
int bbox_potentiometer_knob = 0;

// Don't forget to actually create a datum of the type defined in the header!
bbox_packet busyboxdata_present;
bbox_packet busyboxdata_lastsent;

void setup() 
{
  //Stuff for I2C chatter
  Wire.begin(I2CADDRESS_BBOX); // join i2c bus (address optional for master)
  //Wire.onRequest(requestEvent); // register event

  //initialize switches as inputs
  pinMode(BUTTON_BLU,     INPUT_PULLUP);
  pinMode(ROCKER_HORIZ,   INPUT_PULLUP);
  pinMode(ROCKER_VERTI,   INPUT_PULLUP);
  pinMode(TOGGLE_HORIZ,   INPUT_PULLUP);
  pinMode(TOGGLE_VERTI,   INPUT_PULLUP);
  pinMode(POTENTIOM_LEVER, INPUT);
  pinMode(POTENTIOM_KNOB,  INPUT);

  //Initialize serial port for potentiometer
  Serial.begin(9600);
}

void loop() 
{
  //Read switch states
  present_bbox_button_blu    = digitalRead(BUTTON_BLU);
  present_bbox_rocker_horiz  = digitalRead(ROCKER_HORIZ);
  present_bbox_rocker_verti  = digitalRead(ROCKER_VERTI);
  present_bbox_toggle_horiz  = digitalRead(TOGGLE_HORIZ);
  present_bbox_toggle_verti  = digitalRead(TOGGLE_VERTI);
  present_bbox_potentiometer_lever     = analogRead(POTENTIOM_LEVER);
  present_bbox_potentiometer_knob     = analogRead(POTENTIOM_KNOB);

  Serial.print("Btn: \t");
  if (present_bbox_button_blu == HIGH) 
  {
    Serial.print("--\t"); // N.B. that this is inverted due to pulldown. -JA
  }
  else
  {
    Serial.print("ON\t");
    bbox_button_blu_press_recorded++;
  }
  
  Serial.print("RockH: \t");
  if (present_bbox_rocker_horiz == HIGH) 
  {
    Serial.print("ON\t");
    bbox_rocker_horiz  = 1;
  }
  else
  {
    Serial.print("--\t");
    bbox_rocker_horiz  = 0;
  }
  
  Serial.print("RockV: \t");
  if (present_bbox_rocker_verti == HIGH) 
  {
    Serial.print("ON\t");
    bbox_rocker_verti  = 1;
  }
  else
  {
    Serial.print("--\t");
    bbox_rocker_verti  = 0;
  }
  
  Serial.print("ToggH: \t");
  if (present_bbox_toggle_horiz == HIGH) 
  {
    Serial.print("ON\t");
    bbox_toggle_horiz  = 1;
  }
  else
  {
    Serial.print("--\t");
    bbox_toggle_horiz  = 0;
  }
  
  Serial.print("ToggV: \t");
  if (present_bbox_toggle_verti == HIGH) 
  {
    Serial.print("ON\t");
    bbox_toggle_verti  = 1;
  }
  else
  {
    Serial.print("--\t");
    bbox_toggle_verti  = 0;
  }

  /*Hack the 0-1023 range of the potentiometer down to 0-100 to reduce data 
  and clean out noise. 100% my idea. I'm so smart. -JA*/
  present_bbox_potentiometer_lever = present_bbox_potentiometer_lever / potentiometer_truncator_hack;
  bbox_potentiometer_lever = (int)present_bbox_potentiometer_lever;
  Serial.print("Pot: ");
  Serial.print((int)present_bbox_potentiometer_lever);
  Serial.print("Pot: ");
  Serial.print((int)present_bbox_potentiometer_knob);
   
  /*Clean up the end of the line and wait a bit.*/
  Serial.println("");
/*  if ( busyboxdata_present == busyboxdata_lastsent)
  {
    Serial.println("No change since last message.\t" + String(busyboxdata_present.bbox_button_blu_press_recorded));
    //Wait until the button is depressed again before resetting the button-press-recorded var.
    if (present_bbox_button_blu == HIGH)
    {
      bbox_button_blu_press_recorded = 0;
    }
  }
  else
  {
    Serial.println("Change detected!\t" + String(busyboxdata_present.bbox_button_blu_press_recorded));   
requestEvent(); 
  } */
  delay(250);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
/*void requestEvent() 
{
  if (!( busyboxdata_present == busyboxdata_lastsent))
  {
    //If we have something to send, transmit the data, and stow our current values into the persistent comparator.  
    Wire.write((byte*)&busyboxdata_present, sizeof(bbox_packet));
    busyboxdata_lastsent = busyboxdata_present;
  }
  else
  {
    //If we don't have something to send, send back a series of '1's (TODO remove later)
    for (int q = 0; q++; q < sizeof(bbox_packet))
    {
      Wire.write(1);  
    }
  }
}*/
