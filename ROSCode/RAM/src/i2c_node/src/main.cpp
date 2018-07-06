#include <errno.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <i2c_node/ram_funcs.h>
#include <i2c_node/ram_registers.h>

struct ArduinoRegisters
{
    ADA_input_register ada_input_register;
    ADA_output_register ada_output_register;
};

int main( int argc, char *argv[] )
{
    bool shutdown = false;
    std::chrono::seconds sleep_duration(2);
    /*
    int arduino_handler = -1;
    while( arduino_handler == -1 )
    {
        arduino_handler = wiringPiI2CSetup( I2CADDRESS_ADA );
        if( arduino_handler == -1 )
            std::cout << "ADA not detected" << std::endl;
        else
            std::cout << "ADA detected" << std::endl;
    }

    ArduinoRegisters registers;
    unsigned long &internal_time_register = registers.ada_output_register.time_register;

    while( !shutdown )
    {
        ADA_output_register new_read;
        read( arduino_handler, (byte *)&new_read, sizeof( ADA_output_register));
        if( new_read.verifyCheckSums() )
        {
            // check differences for setting flags
            registers.ada_output_register = new_read;
            std::cout << "Read CheckSum: Success" << std::endl;
            std::cout << "Arduino's Time is: " << registers.ada_output_register.time_register << std::endl;
        }
        else
            std::cout << "Read CheckSum: Failed" << std::endl;
        std::this_thread::sleep_for( sleep_duration );
        registers.ada_input_register.setCheckSums();
        ssize_t n = write( arduino_handler, (byte *)&registers.ada_input_register, 200);
        std::cout << "Reason: " <<  strerror(errno) << std::endl;
        std::this_thread::sleep_for( sleep_duration );
    }
     */

    int serial_handle = -1;
    while( serial_handle == -1 )
    {
        serial_handle = serialOpen( "/ttyAMA0", 9600 );
    }

    while( !shutdown )
    {
        serialPrintf( serial_handle, "This is a test" );
        std::this_thread::sleep_for( sleep_duration );
    }

    return 0;
}
