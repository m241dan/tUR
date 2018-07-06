
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
#include <ram_network/NetworkNode.h>
#include <ros/ros.h>

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
const double        refreshRate     = 0.05; // 20Hz

#endif

