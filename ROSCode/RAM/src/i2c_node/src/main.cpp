#include <thread>
#include <chrono>
#include <iostream>
#include <array>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <i2c_node/ram_funcs.h>

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
        int data = wiringPiI2CReadReg8( arduino_handler, 0x00 );
        if( data == 0x31 )
        {
            byte buf[sizeof(image_packet)] = { 0 };
            buf[0] = (byte)data;
            for( int x = 1; x < sizeof(image_packet); x++ )
                buf[x] = (byte)wiringPiI2CReadReg8( arduino_handler, 0x00 );

            image_packet received(buf);
            std::cout << "imagepacket_position[" << received.imagepacket_position << "]" << std::endl;
            std::cout << "imagepacket_photo_number[" << received.imagepacket_photo_number << "]" << std::endl;
            std::cout << "imagepacket_meat_0[" << received.imagepacket_meat[0] << "]" << std::endl;
            std::cout << "imagepacket_meat_50[" << received.imagepacket_meat[50] << "]" << std::endl;
        }
        std::cout << "Data returned is " << data << std::endl;
        std::this_thread::sleep_for( sleep_duration );
    }

    return 0;
}
