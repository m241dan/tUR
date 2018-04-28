//
// Created by korisd on 4/28/18.
//

#ifndef ARM_CONTROLLER_SHARED_TYPES_H
#define ARM_CONTROLLER_SHARED_TYPES_H

#include <geometry_msgs/Pose.h>
#include <dynamixel_workbench_msgs/XH.h>
#include <string.h>

enum
{
    ROTATION_SERVO, SHOULDER_SERVO,
    ELBOW_SERVO, WRIST_SERVO, MAX_SERVO
};

typedef struct inputs_table
{
    std::vector<geometry_msgs::Pose> waypoint_queue;
    dynamixel_workbench_msgs::XH servos[MAX_SERVO];
    geometry_msgs::Pose current_position;
} InputsTable;

typedef struct servo_command
{
    uint8_t id = 0;
    std::string command = "";
    int32_t value = 0;
} ServoCommand;

#endif //ARM_CONTROLLER_SHARED_TYPES_H
