#ifndef goat_funcs_h
#define goat_funcs_h

#include <SD.h>
#include "Arduino.h"
#include "hasp_types.h"
#include "HardwareSerial.h"

void sendData( HardwareSerial &serial, byte *data, int length );
void assignEntry( char *dst, const char *src, int length, bool from_uplink );
void receiveData( HardwareSerial &serial, byte (&buffer)[MAX_BUF], int &index, SENSOR_READING *reading, GROUND_COMMAND *com, GROUND_GTP *gtp );
String getNextFile( String name );

#endif
