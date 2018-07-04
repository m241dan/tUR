#include <thread>
#include <chrono>
#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>

const int ARDUINO_ADDR = 0x04;

int main( int argc, char *argv[] )
{
    bool shutdown = false;
    int arduino_handler = -1;
    std::chrono::seconds sleep_duration(2);
    while( arduino_handler == -1 )
    {
        arduino_handler = wiringPiI2CSetup( ARDUINO_ADDR );
        std::cout << "I2C Device: " << ARDUINO_ADDR << " not detected." << std::endl;
    }

    while( !shutdown )
    {
        int data = wiringPiI2CRead( arduino_handler );
        std::cout << "Data returned is " << data << std::endl;
        std::this_thread::sleep_for( sleep_duration );
    }

    return 0;
}
