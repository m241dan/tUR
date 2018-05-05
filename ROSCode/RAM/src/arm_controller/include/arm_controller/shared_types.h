//
// Created by korisd on 4/28/18.
//

#ifndef ARM_CONTROLLER_SHARED_TYPES_H
#define ARM_CONTROLLER_SHARED_TYPES_H

#include <geometry_msgs/Pose.h>
#include <dynamixel_workbench_msgs/XH.h>
#include <string.h>

#define OFF_STATE "off_state"
#define PAUSE_STATE "pause_state"
#define WAITING_STATE "waiting_state"
#define GO_STATE "go_state"
#define GO_SYNCHRONIZED_STATE "go_synchronized_state"

#define MAX_VELOCITY 20
#define PID_I_GAIN 150


enum
{
    ROTATION_SERVO = 0, SHOULDER_SERVO,
    ELBOW_SERVO, WRIST_SERVO, MAX_SERVO
};

enum
{
    OFF_MODE, PAUSE_MODE, WAITING_MODE,
    GO_MODE, GO_SYNCHRONIZED_MODE,
    MAX_MODE
};

/* [servo][0:min|1:max] */

const uint16_t servo_min_max[MAX_SERVO][2] = { { 1200, 2826 },   /* rotation servo */
                                         { 2000, 4065 },   /* shoulder servo */
                                         { 310, 3524 },   /* elbow servo */
                                         { 1430, 3362 } }; /* wrist servo */

const std::string mode_state_strings[MAX_MODE] = {
        OFF_STATE, PAUSE_STATE, WAITING_STATE,
        GO_STATE, GO_SYNCHRONIZED_STATE
};
typedef struct inputs_table
{
    std::vector<geometry_msgs::Pose> waypoint_queue;
    dynamixel_workbench_msgs::XH servos[MAX_SERVO];
    geometry_msgs::Pose current_position;
    uint8_t desired_mode = OFF_MODE;
} InputsTable;

typedef struct servo_command
{
    uint8_t id = 0;
    std::string command = "";
    int32_t value = 0;
} ServoCommand;

#endif //ARM_CONTROLLER_SHARED_TYPES_H
