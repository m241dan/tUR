#include "ram_funcs.h";

//Assign variable names to switch and LED pins for convenience
const int BUTTON_BLU    = 24;     //blue button pin
const int ROCKER_HORIZ  = 23;   //horizontal rocker switch (black)
const int ROCKER_VERTI  = 22;   //vertical rocker switch (black)
const int TOGGLE_HORIZ  = 5;   //horizontal toggle switch (metal)
const int TOGGLE_VERTI  = 6;   //vertical toggle switch (metal)
const int POTENTIOM_LEVER = A0;        //potentiometer (ANALOG pin, not digital)
const int POTENTIOM_KNOB = A1;         //potentiometer (ANALOG pin, not digital)

//Define variables for pin states, set to LOW
int present_bbox_rocker_horiz  = 0;
int present_bbox_rocker_verti  = 0;
int present_bbox_toggle_horiz  = 0;
int present_bbox_toggle_verti  = 0;
int present_bbox_button_blu    = 0;
int present_bbox_button_blu_press_recorded    = 0;
int present_bbox_flap;
int present_bbox_potentiometer_lever = 0;
int present_bbox_potentiometer_knob = 0;
float potentiometer_truncator_hack = 10.23;

//Build a data structure to condense all bbox information into an easily-checkable blurb
/*TODO
 * TODO
 * TODO
 * swap this out for an instantiation of the struct already canned in the header file
 * also maybe move the comparator over to there?
 */
struct busyboxdata
{
  byte bbox_rocker_horiz  = 0;
  byte bbox_rocker_verti  = 0;
  byte bbox_toggle_horiz  = 0;
  byte bbox_toggle_verti  = 0;
  byte bbox_button_blu = 0;
  byte bbox_button_blu_press_recorded = 0;
  byte bbox_flap = 0;
  byte bbox_potentiometer_lever = 0;
  byte bbox_potentiometer_knob = 0;
  /* DIY function to compare structs of type 'busyboxdata' to one another since struct comparison isn't native. */
  /* TODO
   *  TODO
   *  TODO
   *  TODO
   *  replace with memcmp
   *  and then QA line by line with DK
   *  TODO
   */
  bool operator==(const busyboxdata& rhs)
  {
    bool same = true;
    if (bbox_rocker_horiz != rhs.bbox_rocker_horiz)
    {
      same = false;
    }
    if (bbox_rocker_verti != rhs.bbox_rocker_verti)
    {
      same = false;
    }
    if (bbox_toggle_horiz != rhs.bbox_toggle_horiz)
    {
      same = false;
    }
    if (bbox_toggle_verti != rhs.bbox_toggle_verti)
    {
      same = false;
    }
    if (bbox_button_blu_press_recorded != rhs.bbox_button_blu_press_recorded)
    {
      same = false;
    }
    if (bbox_flap != rhs.bbox_flap)
    {
      same = false;
    }
    if (bbox_potentiometer_lever != rhs.bbox_potentiometer_lever)
    {
      same = false;
    }
    if (bbox_potentiometer_knob != rhs.bbox_potentiometer_knob)
    {
      same = false;
    }
    return same;
  }
};

// Don't forget to actually create a datum of the type defined above!
busyboxdata busyboxdata_present;
busyboxdata busyboxdata_lastsent;

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
  present_bbox_button_blu    = digitalRead(BUTTON_BLU);
  present_bbox_rocker_horiz  = digitalRead(ROCKER_HORIZ);
  present_bbox_rocker_verti  = digitalRead(ROCKER_VERTI);
  present_bbox_toggle_horiz  = digitalRead(TOGGLE_HORIZ);
  present_bbox_toggle_verti  = digitalRead(TOGGLE_VERTI);
  present_bbox_potentiometer_lever     = analogRead(POTENTIOM_LEVER);

  Serial.print("BBox ");
  Serial.print("Btn: \t");
  if (present_bbox_button_blu == HIGH) 
  {
    Serial.print("ON\t");
    /*
     * TODO
     * TODO
     * TODO
     * Implement logic that checks if a button press was queued up and if so stow it into button_press_recorded etc.
     * TODO
     */
  }
  else
  {
    Serial.print("--\t");
  }
  
  Serial.print("RockH: \t");
  if (present_bbox_rocker_horiz == HIGH) 
  {
    Serial.print("ON\t");
    busyboxdata_present.bbox_rocker_horiz  = 1;
  }
  else
  {
    Serial.print("--\t");
    busyboxdata_present.bbox_rocker_horiz  = 0;
  }
  
  Serial.print("RockV: \t");
  if (present_bbox_rocker_verti == HIGH) 
  {
    Serial.print("ON\t");
    busyboxdata_present.bbox_rocker_verti  = 1;
  }
  else
  {
    Serial.print("--\t");
    busyboxdata_present.bbox_rocker_verti  = 0;
  }
  
  Serial.print("ToggH: \t");
  if (present_bbox_toggle_horiz == HIGH) 
  {
    Serial.print("ON\t");
    busyboxdata_present.bbox_toggle_horiz  = 1;
  }
  else
  {
    Serial.print("--\t");
    busyboxdata_present.bbox_toggle_horiz  = 0;
  }
  
  Serial.print("ToggV: \t");
  if (present_bbox_toggle_verti == HIGH) 
  {
    Serial.print("ON\t");
    busyboxdata_present.bbox_toggle_verti  = 1;
  }
  else
  {
    Serial.print("--\t");
    busyboxdata_present.bbox_toggle_verti  = 0;
  }

  /* Bundle up the data into struct. */
  
  
  /*Hack the 0-1023 range of the potentiometer down to 0-100 to reduce data 
  and clean out noise. 100% my idea. I'm so smart. -JA*/
  present_bbox_potentiometer_lever = present_bbox_potentiometer_lever / potentiometer_truncator_hack;
  busyboxdata_present.bbox_potentiometer_lever = (int)present_bbox_potentiometer_lever;
  Serial.print("Pot: ");
  Serial.print((int)present_bbox_potentiometer_lever);
  
  /*Clean up the end of the line and wait a bit.*/
  Serial.println("");
  if ( busyboxdata_present == busyboxdata_lastsent)
  {
    Serial.println("No change since last message.");
  }
  else
  {
    Serial.println("Change detected!");
    //TODO TODO SEND THE DATA HERE
    busyboxdata_lastsent = busyboxdata_present;
  }
  delay(500);
}
