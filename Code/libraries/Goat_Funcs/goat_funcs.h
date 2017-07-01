 #ifndef goat_funcs_h
#define goat_funcs_h

#include "Arduino.h"
#include "hasp_types.h"
#include "HardwareSerial.h"

void sendData( HardwareSerial &serial, byte *data, int length );
void assignEntry( char *dst, const char *src, int length, bool from_uplink = false );
TRANS_TYPE receiveData( HardwareSerial &serial, byte (&buffer)[256], unsigned int &index );
void bufferToReading( byte (&buffer)[256], SENSOR_READING *reading );
void bufferToCommand( byte (&buffer)[256], GROUND_COMMAND *com );
void bufferToGTP( byte (&buffer)[256], GTP_DATA *gtp );
void resetBuffer( byte (&buffer)[256], unsigned int &index );
String getNextFile( String name );

void sendCommand( HardwareSerial &serial, GROUND_COMMAND &com );

#endif
