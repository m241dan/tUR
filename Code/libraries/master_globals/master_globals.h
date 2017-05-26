#ifndef master_globals_h
#define master_globals_h

#include <Adafruit_BME280.h>
#include <Adafruit_AM2315.h>
#include "Spec.h"
#include "hasp_types.h"
#include "Arduino.h"

//GOAT macros

#define SD_PIN 40
#define BME_PIN 41
#define TEMP_BUS 42
#define MAX_BUF 256
#define C_TIME() ( String( millis() / 1000.0F ).c_str() )

//GOAT master globals

extern Spec so2;
extern Spec no2;
extern Spec o3;
extern Adafruit_BME280 bme;
extern Adafruit_AM2315 dongle;
extern String log_name;
extern GROUND_COMMAND master_command;
extern GROUND_COMMAND slave_command;
extern GTP_DATA current_gtp;
extern SENSOR_READING master_reading;
extern SENSOR_READING slave_reading;
extern bool pump_on;
extern bool take_readings;
extern byte receive_buffer_ground[MAX_BUF];
extern unsigned int ground_index;
extern byte receive_buffer_slave[MAX_BUF];
extern unsigned int slave_index;

#endif
