#ifndef state_h
#define state_h

#include "Arduino.h"
#include "hasp_types.h"
#include "HardwareSerial.h"
#include "goat_funcs.h"
#include "master_types.h"
#include "pump_controller.h"
#include <SD.h>

typedef enum : byte
{
    RECEIVE_GROUND, RECEIVE_SLAVE, DOWNLINK_GROUND, REQUEST_SLAVE_READING,
    COMMAND_HANDLER, TIMER_HANDLER, SAMPLE, NONE_SPECIFIC
} STATE_ID;

#define MAX_STATE (NONE_SPECIFIC)

class state
{
    public:
        state::state( REFS_TABLE &r ) : refs(r) {}
        virtual STATE_ID run() { return; }
    protected:
        REFS_TABLE &refs;
};

class receive_ground : public state
{
    public:
        //functions
        receive_ground( REFS_TABLE &r ) : state( r ) {}
        virtual STATE_ID run();
};

class receive_slave : public state
{
    public:
        //functions
        receive_slave( REFS_TABLE &r ) : state( r ) {}
        virtual STATE_ID run();
};

class downlink_ground : public state
{
    public:
        //functions
        downlink_ground( REFS_TABLE &r ) : state( r ) {}
        virtual STATE_ID run();
    private:
        //functions
        void prepareReading( SENSOR_READING &reading );
        void writeSD( SENSOR_READING &reading );
};

class request_slave_reading : public state
{
    public:
        //functions
        request_slave_reading( REFS_TABLE &r ) : state( r ) {}
        virtual STATE_ID run();
};

class command_handler : public state
{
    public:
        //functions
        command_handler( REFS_TABLE &r ) : state( r ) {}
        virtual STATE_ID run();
};

class timer_handler : public state
{
    public:
        //functions
        timer_handler( REFS_TABLE &r ) : state( r ) {}
        virtual STATE_ID run();
};

class sample : public state
{
    public:
        //functions
        sample( REFS_TABLE &r ) : state( r ) {}
        virtual STATE_ID run();
};

#endif


