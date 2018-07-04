#ifndef hasp_arduino_sysclock_h
#define hasp_arduino_sysclock_h

class ArduinoSysClock
{
    public:
        ArduinoSysClock( unsigned long &cr );
        void updateClock( unsigned long present_millis );
        void syncClock( unsigned long sync_time, unsigned long present_millis );
    private:
        unsigned long sync_to;
        unsigned long last_received;
        unsigned long &clock_register;
};

#endif
