#include "ram_funcs.h"

ArduinoSysClock::ArduinoSysClock( unsigned long &cr ) : clock_register(cr)
{
    clock_register = 0;
}

void ArduinoSysClock::updateClock( unsigned long present_millis )
{
    unsigned long time_elapsed = 0UL;

    time_elapsed    = present_millis / 1000UL - last_received;
    clock_register  = sync_to + time_elapsed;
}

void ArduinoSysClock::syncClock( unsigned long sync_time, unsigned long present_millis )
{
    last_received = present_millis / 1000;
    sync_to = sync_time;
}
