#ifndef master_globals_h
#define master_globals_h

//GOAT macros

#define SD_PIN 40
#define BME_PIN 41
#define TEMP_BUS 42
#define MAX_BUF 512
#define C_TIME() ( String( millis() / 1000.0F ).c_str() )

//GOAT master globals

extern Spec so2( SPEC_O2, A0, A1, A2, 43.54 );
extern Spec no2( SPEC_NO2, A3, A4, A5, 43.54 );
extern Spec o3( SPEC_O3, A6, A7, A8, 43.54 );
extern Adafruit_BME280 bme( BME_PIN );
extern Adafruit_AM2315 dongle;
extern String log_name;
extern GROUND_COMMAND current_command;
extern GROUND_COMMAND slave_command;
extern GROUND_GTP current_gtp;
extern SENSOR_READING master_reading;
extern SENSOR_READING slave_reading;
extern bool pump_on;
extern bool take_readings;
extern byte receive_buffer_ground[MAX_BUF];
extern int ground_index;
extern byte receive_buffer_slave[MAX_BUF];
extern int slave_index;

#endif
