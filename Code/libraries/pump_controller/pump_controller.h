#ifndef pump_controller_h
#define pump_controller_h

#include "Arduino.h"

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

