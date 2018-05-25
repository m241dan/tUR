//
// Created by korisd on 5/25/18.
//

#ifndef ARM_MOTION_ARM_MOTION_NODE_H
#define ARM_MOTION_ARM_MOTION_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "arm_motion/ArmMotionAction.h"

enum
{
    ROTATION_SERVO = 0, SHOULDER_SERVO,
    ELBOW_SERVO, WRIST_SERVO, MAX_SERVO
};

#define MAX_VELOCITY 20
#define PID_I_GAIN 300
#define MAX_COMMAND 51

typedef struct servo_command
{
    uint8_t id = 0;
    std::string command = "";
    int32_t value = 0;
} ServoCommand;

const std::string valid_commands[MAX_COMMAND] = {
        "Model_Number", "Firmware_Version", "ID",
        "Baud_Rate", "Return_Delay_Time", "Drive_Mode",
        "Operating_Mode", "Secondary_ID", "Protocol_Version",
        "Homing_Offset", "Moving_Threshold", "Temperature_Limit",
        "Max_Voltage_Limit", "Min_Voltage_Limit", "PWM_Limit",
        "Current_Limit", "Acceleration_Limit", "Velocity_Limit",
        "Max_Position_Limit", "Min_Position_Limit", "Shutdown",
        "Torque_Enable", "LED", "Status_Return_Level",
        "Registered_Instruction", "Hardware_Error_Status", "Velocity_I_Gain",
        "Velocity_P_Gain", "Position_D_Gain", "Position_I_Gain",
        "Position_P_Gain", "Feedforward_2nd_Gain", "Feedforward_1st_Gain",
        "Bus_Watchdog", "Goal_PWM", "Goal_Current",
        "Goal_Velocity", "Profile_Acceleration", "Profile_Velocity",
        "Goal_Position", "Realtime_Tick", "Moving",
        "Moving_Status", "Present_PWM", "Present_Current",
        "Present_Velocity", "Present_Position", "Velocity_Trajectory",
        "Position_Trajectory", "Present_Input_Voltage", "Present_Temperature"
};
#endif //ARM_MOTION_ARM_MOTION_NODE_H
