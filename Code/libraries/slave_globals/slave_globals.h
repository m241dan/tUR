#ifndef slave_globals_h
#define slave_globals_h

#include <Adafruit_BME280.h>
#include <Adafruit_AM2315.h>
#include "Spec.h"
#include "hasp_types.h"
#include "Arduino.h"

//GOAT slave macros

#define BME_PIN 53
#define MAX_BUF 256
#define C_TIME() ( String( millis() / 1000.0F ).c_str() )

//GOAT slave globals

extern Spec so2;
extern Spec no2;
extern Spec o3;
extern Adafruit_BME280 bme;
extern GROUND_COMMAND current_command;
extern SENSOR_READING slave_reading;
extern byte receive_buffer[MAX_BUF];
extern unsigned int buffer_index;
extern String bme_status;

#endif
