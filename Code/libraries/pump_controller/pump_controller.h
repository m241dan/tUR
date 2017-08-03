#ifndef pump_controller_h
#define pump_controller_h

#include "Arduino.h"

typedef enum : byte
{
   PUMP_ON_AUTO, PUMP_OFF_AUTO, PUMP_OFF_PRESSURE, PUMP_OFF_TEMP, PUMP_ON_MANUAL, PUMP_OFF_MANUAL, MAX_PUMP
} PUMP_STATUS;

const char *const pump_status_string[MAX_PUMP] = { "P:ON AUTO", "P:OFF AUTO", "P:OFF PRESSURE", "P:OFF TEMP", "P:ON MANUAL", "P:OFF MANUAL" };

class pump_controller
{
    public:
        //functions
        pump_controller( uint8_t pin );
        void on();
        void off();
    private:
        //vars
        uint8_t pump_pin;
};

#endif
