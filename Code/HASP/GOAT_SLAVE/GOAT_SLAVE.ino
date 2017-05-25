#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_SI5351.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AM2315.h>
#include "Spec.h"
#include "hasp_types.h"
#include "goat_funcs.h"
#include "slave_globals.h"

void setup()
{
    setupSlaveSerials();
    setupSlaveGlobals();
    setupSlaveSensors();
}

void loop()
{

}
