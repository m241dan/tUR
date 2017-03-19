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

#endif
