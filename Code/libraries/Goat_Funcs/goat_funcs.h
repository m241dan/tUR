#ifndef goat_funcs_h
#define goat_funcs_h

#include "Arduino.h"
#include "hasp_types.h"
#include "HardwareSerial.h"

void sendData( HardwareSerial &serial, byte *data, int length );
void assignEntry( char *dst, const char *src, int length, bool from_uplink = false );
TRANS_TYPE receiveData( HardwareSerial &serial, byte (&buffer)[256], unsigned int &index, SENSOR_READING *reading, GROUND_COMMAND *com, GTP_DATA *gtp );
String getNextFile( String name );

inline void sendCommand( HardwareSerial &serial, GROUND_COMMAND &com );

#endif
