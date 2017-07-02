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
#define PUMP_PIN 47
#define MAX_BUF 256
#define LOG_NAME "GOAT"
#define FIFTEEN_MINUTES ( 60000 * 15 )
#define C_TIME() ( String( millis() / 1000.0F ).c_str() )

//GOAT master types

typedef struct sensor_table
{
    sensor_table() : so2( SPEC_SO2, A0, A1, A2, 43.54 ),
                     no2( SPEC_NO2, A3, A4, A5, 43.53 ),
                     o3( SPEC_O3, A6, A7, A8, 43.54 ),
                     bme( BME_PIN ) {}
    Spec so2;
    Spec no2;
    Spec o3;
    Adafruit_BME280 bme;
    Adafruit_AM2315 dongle;
} SENSOR_TABLE;

typedef struct readings_table
{
    readings_table() { memset( &master, 0, sizeof( master ) );
                       memset( &slave, 0, sizeof( slave ) );
                       memset( &gtp, 0, sizeof( gtp ) ); }
    SENSOR_READING master;
    SENSOR_READING slave;
    GTP_DATA gtp;
} READINGS_TABLE;

typedef struct status_table
{
    status_table() : log_name(""), pump_on(false), pump_auto(true),
                     reading_status(""), reading_auto(true), sd_status(""),
                     bme_status(""), am2315_status(""), which_bank(1) {}
    String log_name;
    bool pump_on;
    bool pump_auto;
    String reading_status;
    bool reading_auto;
    String sd_status;
    String bme_status;
    String am2315_status;
    byte which_bank;
} STATUS_TABLE;

typedef struct receive_buffers
{
    receive_buffers() : ground_index(0), slave_index(0) {
                        memset( &ground[0], 0, MAX_BUF );
                        memset( &slave[0], 0, MAX_BUF ); }
    byte ground[MAX_BUF];
    unsigned int ground_index;
    byte slave[MAX_BUF];
    unsigned int slave_index;
} RECEIVE_BUFFERS;

typedef struct timer_table
{
    unsigned long long downlink_schedule = 0;
    unsigned long long gtp_time = 0;
    unsigned long long gtp_received_at = 0;
    unsigned long long pump_timer = 0;
    int slave_wait_sanity = 0;
} TIMER_TABLE;

typedef struct data_set
{
    double so2_total = 0;
    double no2_total = 0;
    double o3_total = 0;
    double temp_total = 0;
    double humidity_total = 0;
    double pressure_total = 0;
    double ext_temp_total = 0;
    double ext_humidity_total = 0;
    unsigned int super_sample = 0;
} DATA_SET;

typedef class pump_controller
{
    public:
        //functions
        pump_controller( uint8_t pin ) : pump_pin(pin)
            {
                pinMode( pump_pin, OUTPUT );
                digitalWrite( pump_pin, LOW );
            }
        bool pump_on()
            {
                digitalWrite(
            }
        bool pmup_off();
    private:
        //vars
        uint8_t pump_pin;
} PUMP_CONTROLLER;
#endif
