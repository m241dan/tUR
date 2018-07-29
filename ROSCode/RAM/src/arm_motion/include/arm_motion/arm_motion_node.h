//
// Created by korisd on 5/25/18.
//

#ifndef ARM_MOTION_ARM_MOTION_NODE_H
#define ARM_MOTION_ARM_MOTION_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <std_msgs/UInt8.h>
#include <dynamixel_workbench_msgs/XH.h>
#include <arm_motion/ManualWaypoint.h>
#include <arm_motion/ServoChange.h>
#include <rosgraph_msgs/Clock.h>
#include <arm_motion/ArmInfo.h>
#include <arm_motion/MotionData.h>
#include <arm_motion/TrialData.h>
#include <arm_motion/StartTrial.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>


/*
 * Servo Info
 * For new servos, add them here
 */
enum
{
    ROTATION_SERVO = 0, SHOULDER_SERVO,
    ELBOW_SERVO, WRIST_SERVO, WRIST_ROT_SERVO,
//    GRIPPER_SERVO,
    MAX_SERVO
};

#define MAX_ARM_SERVO 3

enum
{
    DISCRETE_W, DISCRETE_R, VISION, SERVO_R, SERVO_ABSOLUTE
};

const std::string servo_topic_names[MAX_SERVO] = {
        "servo_info/rotation", "servo_info/shoulder",
        "servo_info/elbow", "servo_info/wrist",
        "servo_info/wrist_rotation"
};

const std::string servo_names[MAX_SERVO] = {
        "rotation servo", "shoulder servo",
        "elbow servo", "wrist servo",
        "wrist rotation servo"
};
#define MAX_COMMAND 51
const double length1 = 2.6;
const double length2 = 15.465;
const double length3 = 8.927;
const double length4 = 13.00;
const char * const ram_scripts = "RAM_SCRIPTS";
const uint8_t shadow_id = 15;
const int32_t MAX_VELOCITY = 20;
const int32_t PID_P_GAIN = 900;
const int32_t PID_I_GAIN = 300;
const int32_t PID_D_GAIN = 0;
const int32_t VELOCITY_PID_P_GAIN = 1920;
const int32_t VELOCITY_PID_I_GAIN = 100;
const int32_t PROFILE_ACC = 1;

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

const std::string serial_service_string( "/synchronizer/serial" );
const std::string servo_loop_string    ( "/synchronizer/servos" );
const std::string i2c_loop_string      ( "/synchronizer/i2c" );

#endif //ARM_MOTION_ARM_MOTION_NODE_H
