#ifndef pump_controller_h
#define pump_controller_h

#include "master_types.h"
#include "Arduino.h"

typedef class pump_controller
{
    public:
        //functions
        pump_controller( uint8_t pin );
        void pump_on();
        void pump_off();
    private:
        //vars
        uint8_t pump_pin;
} PUMP_CONTROLLER;

#endif
