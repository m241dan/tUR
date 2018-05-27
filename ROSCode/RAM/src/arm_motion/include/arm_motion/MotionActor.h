//
// Created by korisd on 5/25/18.
//

#ifndef ARM_MOTION_MOTIONACTOR_H
#define ARM_MOTION_MOTIONACTOR_H

#include "arm_motion/arm_motion_node.h"
#include <actionlib/server/simple_action_server.h>
#include "arm_motion/DynamixelController.h"

class MotionActor
{
    public:
        MotionActor( std::string name, DynamixelController controller ) : action_server( node_handle, name, false ), action_name(name), _controller(controller)
        {
            action_server.registerGoalCallback( boost::bind( &MotionActor::goalCallBack, this ) );
            action_server.registerPreemptCallback( boost::bind( &MotionActor::preemptCallBack, this ) );
            action_timer = node_handle.createTimer( ros::Duration(0.1), boost::bind( &MotionActor::motionChecker, this, _1 ) );
            action_timer.stop();
        }

    protected:
        void goalCallBack()
        {
            /*setup the new motion*/
            rotation_goal = action_server.acceptNewGoal()->rotation;
            shoulder_goal = action_server.acceptNewGoal()->shoulder;
            elbow_goal = action_server.acceptNewGoal()->elbow;
            wrist_goal = action_server.acceptNewGoal()->wrist;
            goal_step = 0;
            action_timer.start();
        }

        void preemptCallBack()
        {

        }

        void motionChecker( const ros::TimerEvent &event )
        {

        };

        void performMotion()
        {
            if( goal_step != rotation_goal.size() )
            {
                ServoCommand com;
                com.command = "Goal_Position";


            }
            else
            {
                ROS_INFO( "%s: attempting to perform a goal[%d}] outside the bounds of the goal vectors.", __FUNCTION__, goal_step );
            }
        }
        /*
         * ROS Stuff
         */
        ros::NodeHandle node_handle;
        /* Action Server */
        actionlib::SimpleActionServer<arm_motion::ArmMotionAction> action_server;
        std::string action_name;
        arm_motion::ArmMotionActionGoal goal;
        arm_motion::ArmMotionActionFeedback feedback;
        arm_motion::ArmMotionActionResult result;
        /* Action Timer */
        ros::Timer action_timer;

        /*
         * Dynamixel Controller
         */
        DynamixelController _controller;

        /* Joint Goals */
        std::vector<sensor_msgs::JointState> rotation_goal;
        std::vector<sensor_msgs::JointState> shoulder_goal;
        std::vector<sensor_msgs::JointState> elbow_goal;
        std::vector<sensor_msgs::JointState> wrist_goal;
        uint8_t goal_step;
};


#endif //ARM_MOTION_MOTIONACTOR_H
