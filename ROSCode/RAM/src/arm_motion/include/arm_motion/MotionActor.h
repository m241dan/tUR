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
            action_timer = node_handle.createTimer( ros::Duration(0.1), boost::bind( &MotionActor::motionMonitor, this, _1 ) );
            action_timer.stop();
        }

    protected:
        void goalCallBack()
        {
            /*setup the new motion*/
            for( int i = 0; i < MAX_SERVO; i++ )
            {
                joint_goals[i] = action_server.acceptNewGoal()->joints[i];
            }
            goal_step = 0;
            goal_max = (uint8_t)joint_goals[0].position.size();
            performMotionStep(); //the first, as soon as it receives the goal
            action_timer.start();
        }

        void preemptCallBack()
        {

        }

        void motionMonitor( const ros::TimerEvent &event )
        {

        };

        bool checkMotionStep()
        {

        }
        bool performMotionStep()
        {
            bool success = true;
            if( goal_step != goal_max )
            {
                for( uint8_t i = 0; i < MAX_SERVO; i++ )
                {
                    uint8_t id = i + (uint8_t)1;
                    uint32_t position = (uint32_t)joint_goals[i].position[goal_step];
                    uint32_t velocity = (uint32_t)joint_goals[i].velocity[goal_step];

                    bool status = _controller.changePosition( id, position );
                    if( !status )
                    {
                        ROS_INFO( "%s: failed to write goal position[%d] to servo[%d]", __FUNCTION__, position, id );
                        success = false;
                    }

                    status = _controller.changeVelocity( id, velocity );
                    if( !status )
                    {
                        ROS_INFO( "%s: failed to write profile velocity[%d] to servo[%d}", __FUNCTION__, velocity, id );
                        success = false;
                    }
                }
            }
            else
            {
                ROS_INFO( "%s: attempting to perform a goal[%d}] outside the bounds of the goal vectors.", __FUNCTION__, goal_step );
                success = false;
            }
            return success;
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
        DynamixelController &_controller;

        /* Joint Goals */
        sensor_msgs::JointState joint_goals[MAX_SERVO];
        uint8_t goal_step;
        uint8_t goal_max;
};


#endif //ARM_MOTION_MOTIONACTOR_H
