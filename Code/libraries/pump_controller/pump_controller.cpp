#include "pump_controller.h"

pump_controller::pump_controller( uint8_t pin ) : pump_pin(pin)
{
    pinMode( pump_pin, OUTPUT );
    digitalWrite( pump_pin, LOW );
}

void pump_controller::pump_on()
{
    digitalWrite( pump_pin, HIGH );
}

void pump_controller::pump_off()
{
    digitalWrite( pump_pin, LOW );
}
