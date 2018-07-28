//
// Created by korisd on 5/25/18.
//

#ifndef ARM_MOTION_MOTIONACTOR_H
#define ARM_MOTION_MOTIONACTOR_H

#include "arm_motion/arm_motion_node.h"
#include <actionlib/server/simple_action_server.h>
#include "arm_motion/DynamixelController.h"
#include <vector>
#include "arm_motion/ArmMotionAction.h"
#include "arm_motion/ServoMotionAction.h"



class MotionActor
{
    public:
        MotionActor( std::string name, DynamixelController &controller );
    protected:
        void goalCallBack();
        void preemptCallBack();
        void motionMonitor( const ros::TimerEvent &event );
        bool checkMotionStep();
        bool performMotionStep();
        /*
         * ROS Stuff
         */
        ros::NodeHandle node_handle;
        /* Action Server */
        actionlib::SimpleActionServer<arm_motion::ArmMotionAction> action_server;
        std::string action_name;
        arm_motion::ArmMotionFeedback feedback;
        arm_motion::ArmMotionResult result;
        /* Action Timer */
        ros::Timer action_timer;
        ros::Timer servo_timer;

        actionlib::SimpleActionServer<arm_motion::ServoMotionAction> servo_server;
        arm_motion::ServoMotionGoal _goal;
        void servoGoalCallback();
        void servoPreemptCallback();
        void servoMonitor( const ros::TimerEvent &event );
        bool checkServoStep();
        bool performServoStep();

        /*
         * Dynamixel Controller
         */
        DynamixelController &_controller;

        /* Joint Goals */
        std::vector<sensor_msgs::JointState> joint_goals;
        int16_t goal_step;
        int16_t goal_max;

        ros::ServiceClient _start_motion;
        ros::ServiceClient _stop_motion;
};


#endif //ARM_MOTION_MOTIONACTOR_H
