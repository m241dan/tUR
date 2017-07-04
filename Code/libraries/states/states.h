#ifndef state_h
#define state_h

#include "Arduino.h"
#include "hasp_types.h"
#include "HardwareSerial.h"
#include "goat_funcs.h"
#include "master_types.h"
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
        virtual STATE_ID void run() { return; }
};

class receive_ground : public state
{
    public:
        //functions
        receive_ground( GROUND_COMMAND &handle, GTP_DATA &gps, HardwareSerial &serial ) : command_handle(handle),
            gtp(gps), ground_serial(serial) {}
        virtual STATE_ID run();
    private:
        //vars
        GROUND_COMMAND &command_handle;
        GTP_DATA &gtp;
        HardwareSerial &ground_serial;
};

class receive_slave : public state
{
    public:
        //functions
        receive_slave( SENSOR_READING &read, HardwareSerial &serial ) : reading(read),
            slave_serial(serial) {}
        virtual STATE_ID run();
    private:
        //vars
        SENSOR_READING &reading;
        HardwareSerial &slave_serial;
};

class downlink_ground : public state
{
    public:
        //functions
        downlink_ground( READINGS_TABLE &tab, DATA_SET &set, STATUS_TABLE &stab, TIMERS_TABLE &ttab, HardwareSerial &serial HardwareSerial *bserial = nullptr ) : readings(tab),
            data(set), statuss(stab), timers(ttab), ground_serial(serial),
            blu_serial(bserial) {}
        virtual STATE_ID run();
    private:
        //functions
        void prepareReading( SENSOR_READING &reading );
        void writeSD( SENSOR_READING &reading );

        //vars
        READINGS_TABLE &readings;
        DATA_SET &data;
        STATUS_TABLE &statuss;
        TIMERS_TABLE &timers;
        HardwareSerial &ground_serial;
        HardwareSerial *blu_serial;
};

class request_slave_reading : public state
{
    public:
        //functions
        request_slave_reading( HardwareSerial &serial ) : slave_serial(serial) {}
        virtual STATE_ID run();
    private:
        //vars
        HardwareSerial &slave_serial;
};

class command_handler : public state
{
    public:
        //functions
        command_handler( GROUND_COMMAND &hand ) : handle(hand) {}
        virtual STATE_ID run();
    private:
        //vars
        GROUND_COMMAND &handle;
};

class timer_handler : public state
{
    public:
        //functions
        timer_handler( TIMER_TABLE &tab, GTP_DATA &g, STATUS_TABLE &s, pump_controller &p ) : timers(tab),
            gtp(p), statuss(s), pump(p) {}
        virtual STATE_ID run();
    private:
        //vars
       GTP_DATA &gtp;
       TIMER_TABLE &timers;
       STATUS_TABLE &statuss;
       pump_controller &pump;
};

class sample : public state
{
    public:
        //functions
        sample( DATA_SET &set, SENSOR_TABLE &tab ) : data(set),
            sensors(tab) {}
        virtual STATE_ID run();
    private:
        //vars
        SENSOR_TABLE &sensors;
        DATA_SET &data;
};

#endif

