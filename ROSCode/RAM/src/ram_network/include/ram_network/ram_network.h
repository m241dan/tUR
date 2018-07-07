
//
// Created by korisd on 7/5/18.
//

#ifndef RAM_NETWORK_H
#define RAM_NETWORK_H

#include <iostream>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <ram_network/ram_funcs.h>
#include <ram_network/ram_registers.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt32.h>
#include <ram_commands.h>

struct ArduinoRegisters
{
    unsigned long           &gps_time_sync;
    unsigned long           &ard_time_sync;
    ADA_input_register      ada_input_register;
    ADA_output_register     ada_output_register;
    BBOX_input_register     bbox_input_register;
    BBOX_output_register    bbox_output_register;
    ArduinoRegisters() : gps_time_sync(ada_input_register.sync_to),
                         ard_time_sync(ada_output_register.time_register) {}
};

struct fdHandles
{
    int serial  = -1;
    int ada     = -1;
    int bbox    = -1;
};

const char *const   serialAddress   = "/dev/ttyAMA0";
const int           serialBaud      = 4800;
const int           serialLimit     = 100; // amount of times to attempt opening a serial connection before failing
const double        refreshRate     = 0.1; // 10Hz

struct gtp
{
    uint8_t header[2] = { '\x01', '\x30' };
    char data[120] = "1234470131.649,$GPGGA,202212.00,3024.7205,N,09110.7264,W,1,06,1.69,00061,M,-025,M,,*51,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,";
    uint8_t terminator[3] = { '\x03', '\x0D', '\x0A' };
};

struct ground_command
{
    uint8_t header[2] = { '\x01', '\x02' };
    uint8_t command[2] = { 0, 0 };
    uint8_t terminator[3] = { '\x03', '\x0D', '\x0A' };
};

#endif

