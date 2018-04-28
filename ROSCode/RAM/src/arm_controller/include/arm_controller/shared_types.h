//
// Created by korisd on 4/28/18.
//

#ifndef ARM_CONTROLLER_SHARED_TYPES_H
#define ARM_CONTROLLER_SHARED_TYPES_H

#include <geometry_msgs/Pose.h>
#include <dynamixel_workbench_msgs/XH.h>

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

#endif //ARM_CONTROLLER_SHARED_TYPES_H
