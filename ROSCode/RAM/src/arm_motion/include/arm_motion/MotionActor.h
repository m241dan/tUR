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
        void motionMonitor();
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
        ros::Subscriber _kinematics_subscriber;

        actionlib::SimpleActionServer<arm_motion::ServoMotionAction> servo_server;
        arm_motion::ServoMotionGoal _goal;
        void servoGoalCallback();
        void servoPreemptCallback();
        void servoMonitor();
        bool checkServoStep();
        bool performServoStep();
        void kinematicsTick( const geometry_msgs::Pose::ConstPtr &pose );

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

        bool _with_force;
};


#endif //ARM_MOTION_MOTIONACTOR_H
