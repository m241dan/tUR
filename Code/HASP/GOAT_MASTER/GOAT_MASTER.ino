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

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_SI5351.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AM2315.h>
#include "Spec.h"
#include "hasp_types.h"
#include "goat_master_funcs.h"
#include "master_globals.h"

void setup()
{
    setupMasterSerials();
    setupMasterGlobals();
    setupMasterSensors();

    delay( 2000 );
    assignEntry( master_reading.time, C_TIME(), sizeof( master_reading.time ) );
    sendData( (byte *)&master_reading, sizeof( master_reading ) );

    //block until we get a response from slave
    while( !Serial1.available() );

    receiveFromSlave();

}

void loop()
{

}

