#ifndef hasp_types_h
#define hasp_types_h

typedef struct ground_command
{
   const unsigned char header[2] = "\x1\x2";
   unsigned char checksum;
   unsigned char command[2];
   const unsigned char terminator[3] = "\x3\xD\xA";
} GROUND_COMMAND;

typedef struct gps_time_position
{
   const unsigned char header[2] = "\x1\x30";
   unsigned char data[119] = "1234470131.649,$GPGGA,202212.00,3024.7205,N,09110.7264,W,1,06,1.69,00061,M,-025,M,,*51,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,";
   const unsigned char terminator[3] = "\x3\xD\xA";
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
   unsigned char peltier_status[30];
   unsigned char sd_status[10];
   unsigned char reading_status[10];
   const unsigned char terminator[2] = "\r\n";
} SENSOR_READING;

//functions
void sendData( byte *data, int length );
void assignEntry( char *dst, const char *src, int length, bool from_uplink );
String getNextFile( String name );

#endif


