#ifndef goat_funcs_h
#define goat_funcs_h

#include "Arduino.h"
#include "hasp_types.h"
#include "HardwareSerial.h"

void sendData( HardwareSerial &serial, byte *data, int length );
void assignEntry( char *dst, const char *src, int length, bool from_uplink = false );
TRANS_TYPE receiveData( HardwareSerial &serial, byte (&buffer)[MAX_BUF], unsigned int &index );
bool bufferToReading( byte (&buffer)[MAX_BUF], SENSOR_READING &reading );
bool bufferToCommand( byte (&buffer)[MAX_BUF], GROUND_COMMAND &com );
bool bufferToGTP( byte (&buffer)[MAX_BUF], GTP_DATA &gtp );
void resetBuffer( byte (&buffer)[MAX_BUF], unsigned int &index );
String getNextFile( String name );

void sendCommand( HardwareSerial &serial, unsigned char command );

#endif
