//
// Created by korisd on 5/25/18.
//

#ifndef ARM_MOTION_MOTIONACTOR_H
#define ARM_MOTION_MOTIONACTOR_H

#include "arm_motion/arm_motion_node.h"
#include <actionlib/server/simple_action_server.h>
#include "arm_motion/DynamixelController.h"
#include <vector>

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
        arm_motion::ArmMotionGoal goal;
        arm_motion::ArmMotionFeedback feedback;
        arm_motion::ArmMotionResult result;
        /* Action Timer */
        ros::Timer action_timer;

        /*
         * Dynamixel Controller
         */
        DynamixelController &_controller;

        /* Joint Goals */
        sensor_msgs::JointState joint_goals[MAX_SERVO];
        uint8_t goal_step;
        uint8_t goal_max;
};


#endif //ARM_MOTION_MOTIONACTOR_H
