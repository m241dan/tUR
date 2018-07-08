//
// Created by korisd on 7/7/18.
//

#ifndef GROUND_NODE_H
#define GROUND_NODE_H

#include <ros/ros.h>
#include <string>
#include <ground_station/HaspCommand.h>
#include <std_msgs/ByteMultiArray.h>

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

#endif //HASP_GROUND_STATION_GROUND_NODE_H
