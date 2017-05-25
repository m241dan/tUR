#ifndef slave_globals_h
#define slave_globals_h

//GOAT slave macros

#define BME_PIN 41
#define MAX_BUF 512
#define C_TIME() ( String( millis() / 1000.0F ).c_str() )

//GOAT slave globals

extern Spec so2( SPEC_SO2, A0, A1, A2, 43.54 );
extern Spec no2( SPEC_NO2, A3, A4, A5, 43.53 );
extern Spec o3( SPEC_O3, A6, A7, A8, 43.56 );
extern Adafruit_BME280 bme( BME_PIN );
extern GROUND_COMMAND current_command;
extern SENSOR_READING;
extern byte receive_buffer[MAX_BUF];

#endif
