/*
 * HASP GOAT: Master Arduino
 * Author: Daniel R. Koris
 * Goal of Program: Issuing commands and receiving readings from the Slave Arduino,
 *                  receiving commands and telemtry from HASP Gondola,
 *                  taking readings from one half of the sensor rig,
 *                  control the Sparkfun pump,
 *                  and thermal controlling.
 * Required Components: Arduino Mega, Adafruit MicroSD Card Breakout Board, Adafruit BME280,
 *                      Spec SO2, Spec O3, Spec NO2, Adafruit Oscillator, Resistor Heaters,
 *                      Thermoelectric Modules
 */

//THIS IS MOSTLY PSEUDO-CODE
//THIS IS MOSTLY PSEUDO-CODE
//THIS IS MOSTLY PSEUDO-CODE
//THIS IS MOSTLY PSEUDO-CODE
//THIS IS MOSTLY PSEUDO-CODE

#include "Spec.h"
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_SI5351.h>

//sensor rig

Spec so2( SPEC_O2, pin, pin, pin );
Spec no2( SPEC_NO2, pin, pin, pin );
Spec o3( SPEC_O3, pin, pin, pin );
Adafruit_BME280 bme( pin );
Adafruit_SI5351 clockgen();
File hasp_log1;
File hasp_log2;

//

void setup()
{

}

void loop()
{

}
