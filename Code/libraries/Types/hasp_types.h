#ifndef hasp_types_h
#define hasp_types_h

typedef struct ground_command
{
   static const char header[2] = "\x1\x2";
   unsigned char checksum;
   unsigned char command;
   static const char terminator[3] = "\x3\xD\xA";
} GROUND_COMMAND;

typedef struct gps_time_position
{
   static const char header[2] = "\x1\x30";
   unsigned char data[119];
   static const char terminator[3] = "\x3\xD\xA";
} GTP_DATA;

typedef struct sensor_readings
{
   unsigned char time[9];
   unsigned char bank[2];
   unsigned char so2_reading[10];
   unsigned char o3_reading[10];
   unsigned char no2_reading[10];
   unsigned char temp_reading[10];
   unsigned char pressure_reading[10];
   unsigned char humidity_reading[10];
   unsigned char pump_status[10];
   unsigned char peltier_stuats[30];
   unsigned char sd_status[10];
   unsigned char reading_status[10];
   unsigned char terminator[2] = "\x3\xD";
}

#endif
