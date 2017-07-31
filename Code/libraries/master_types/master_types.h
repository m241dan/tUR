#ifndef master_globals_h
#define master_globals_h

#include <Adafruit_BME280.h>
#include <Adafruit_AM2315.h>
#include <SD.h>
#include "Spec.h"
#include "hasp_types.h"
#include "Arduino.h"
#include "pump_controller.h"

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
    Spec babi_so2;
    Spec babi_no2;
    Spec babi_o3;
    Adafruit_BME280 babi_bme;
    Spec goat_so2;
    Spec goat_no2;
    Spec goat_o3;
    Adafuit_BME280 goat_bme;
    Adafruit_AM2315 dongle;
} SENSOR_TABLE;

typedef struct readings_table
{
    SENSOR_READING babi_reading;
    SENSOR_READING goat_reading;
    GTP_DATA gtp;
} READINGS_TABLE;

typedef struct status_table
{
    String log_name = "";
    PUMP_STATUS pump_auto = PUMP_OFF_AUTO;
    String reading_status = "";
    bool reading_auto = true;
    String sd_status = "";
    String babi_bme_status = "";
    String am2315_status = "";
    String goat_bme_status = "";
    byte which_bank = 1;
    byte slave_wait_sanity = 1;
    double babi_pressure = 1334.00;
    bool downlink_on = true;
    bool write_sd = true;
} STATUS_TABLE;

typedef struct receive_buffers
{
    byte ground[MAX_BUF];
    unsigned int ground_index = 0;
} RECEIVE_BUFFERS;

typedef struct timer_table
{
    unsigned long long downlink_schedule = 0;
    unsigned long long gtp_time = 0;
    unsigned long long gtp_received_at = 0;
    unsigned long long pump_timer = 0;
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

typedef struct refs_table
{
    SENSOR_TABLE &sensors;
    READINGS_TABLE &readings;
    GROUND_COMMAND &ground_command_handle;
    STATUS_TABLE &statuss;
    RECEIVE_BUFFERS &buffers;
    TIMER_TABLE &timers;
    DATA_SET &sample_set;
    HardwareSerial &ground_serial;
    pump_controller &pump;
} REFS_TABLE;

#endif
