#ifndef hasp_types_h
#define hasp_types_h

#include "Arduino.h"

#define ACKNOWLEDGE '\x06'
#define ARD_RESET '\x20'
#define DOWNLINK_OFF '\x21'
#define DOWNLINK_ON '\x22'
#define STOP_SENSORS '\x23'
#define START_SENSORS '\x24'
#define PUMP_ON '\x25'
#define PUMP_OFF '\x26'
#define DISABLE_SD '\x29'
#define ENABLE_SD '\x2A'
#define REINIT_SD '\x2B'
#define BANK_ONE '\x2E'
#define BANK_TWO '\x2F'

typedef enum
{
   TRANS_INCOMPLETE = 0, TRANS_COMMAND, TRANS_DATA, TRANS_GTP
} TRANS_TYPE;

typedef struct ground_command
{
   unsigned char header[2] = "\x1\x2";
   unsigned char checksum = 0;
   unsigned char command[2] = { 0, 0 };
   unsigned char terminator[3] = "\x3\xD\xA";
} GROUND_COMMAND;

typedef struct sample_gtp
{
   unsigned char header[2] = "\x1\x30";
   unsigned char data[120] = "1234470131.649,$GPGGA,202212.00,3024.7205,N,09110.7264,W,1,06,1.69,00061,M,-025,M,,*51,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,";
   unsigned char terminator[3] = "\x3\xD\xA";
} SAMPLE_GTP;

typedef struct gps_time_position
{
   double utc_time;
   unsigned char NMEA[10];
   double utc_position_time;
   double latitude;
   unsigned char latitude_hemisphere;
   double longitude;
   unsigned char longitude_hemisphere;
   byte position_fix;
   byte num_satelites;
   double horizontal_dilution_precision;
   double msl_altitude;
   int differential_age;
} GTP_DATA;

typedef struct sensor_readings
{
   unsigned char header[2] = "\x1\x21";
   unsigned char time[15];
   unsigned char bank[2];
   unsigned char so2_reading[8];
   unsigned char no2_reading[8];
   unsigned char o3_reading[8];
   unsigned char temp_reading[8];
   unsigned char extt_reading[8];
   unsigned char pressure_reading[10];
   unsigned char humidity_reading[5];
   unsigned char ext_humidity_reading[5];
   unsigned char pump_status[12];
   unsigned char bme_status[10];
   unsigned char am2315_status[10];
   unsigned char sd_status[10];
   unsigned char reading_status[10];
   unsigned char terminator[2] = "\r\n";
} SENSOR_READING;

#endif


