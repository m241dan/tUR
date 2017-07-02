#ifndef state_h
#define state_h

#include "Arduino.h"
#include "hasp_types.h"
#include "HardwareSerial.h"
#include "goat_funcs.h"

typedef enum : byte
{
    RECEIVE_GROUND, RECEIVE_SLAVE, DOWNLINK_GROUND, REQUEST_SLAVE_READING,
    COMMAND_HANDLER, TIMER_HANDLER, SAMPLE, NONE_SPECIFIC
} STATE_ID;

#define MAX_STATE (NONE_SPECIFIC-1)

class state
{
    public:
        virtual void run();
        virtual STATE_ID transition() { return NONE_SPECIFIC; }
};

class receive_ground : public state
{
    public:
        //functions
        receive_ground( GROUND_COMMAND &handle, HardwareSerial &serial ) : command_handle(handle),
            ground_serial(serial) {}
        virtual void run();
    private:
        //vars
        GROUND_COMMAND &command_handle;
        HardwareSerial &ground_serial;
};

class receive_slave : public state
{
    public:
        //functions
        receive_slave( SENSOR_READING &read, HardwareSerial &serial ) : reading(read),
            slave_serial(serial) {}
        virtual void run();
    private:
        //vars
        SENSOR_READING &reading;
        HardwareSerial &slave_serial;
};

class downlink_ground : public state
{
    public:
        //functions
        downlink_ground( READINGS_TABLE tab, DATA_SET &set, HardwareSerial &serial ) : readings(tab),
            data(set), ground_serial(serial) {}
        virtual void run();
    private:
        //functions
        void prepareReading();

        //vars
        READINGS_TABLE &readings;
        DATA_SET &data;
        HardwareSerial &ground_serial;
};

class request_slave_reading : public state
{
    public:
        //functions
        request_slave_reading( HardwareSerial &serial ) : slave_serial(serial) {}
        virtual void run();
    private:
        //vars
        HardwareSerial &slave_serial;
};

class command_handler : public state
{
    public:
        //functions
        command_handler( GROUND_COMMAND &hand ) : handle(hand) {}
        virtual void run();
    private:
        //vars
        GROUND_COMMAND &handle;
};

class timer_handler : public state
{
    public:
        //functions
        timer_handler( TIMER_TABLE &tab ) : timers(tab) {}
        virtual void run();
    private:
        //vars
       TIMER_TABLE &timers;
};

class sample : public state
{
    public:
        //functions
        sample( DATA_SET &set, SENSOR_TABLE &tab ) : data(set),
            sensors(tab) {}
        virtual void run();
    private:
        //vars
        SENSOR_TABLE &sensors;
        DATA_SET &data;
};

#endif

