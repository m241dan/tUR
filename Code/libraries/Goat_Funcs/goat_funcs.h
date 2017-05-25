#ifndef goat_funcs_h
#define goat_funcs_h

void sendData( byte *data, int length );
void sendData1( byte *data, int length );
void assignEntry( char *dst, const char *src, int length, bool from_uplink );
String getNextFile( String name );

void setupMasterSerials( void );
void setupSlaveSerials( void );

void setupMasterGlobals( void );
void setupSlaveGlobals( void );

void setupMasterSensors( void );
void setupSlaveSensors( void );

#endif
