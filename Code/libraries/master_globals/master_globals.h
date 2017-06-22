#ifndef master_globals_h
#define master_globals_h

#include <Adafruit_BME280.h>
#include <Adafruit_AM2315.h>
#include <SD.h>
#include "Spec.h"
#include "hasp_types.h"
#include "Arduino.h"

//GOAT macros

#define SD_PIN 49
#define BME_PIN 53
#define TEMP_BUS 42
#define MAX_BUF 256
#define LOG_NAME "GOAT"
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
extern unsigned long long downlink_schedule;
extern bool new_slave_reading;
extern byte which_bank;
extern String reading_status;
extern String sd_status;
extern String bme_status;
extern String am2315_status;

#endif
