//
// Created by korisd on 5/25/18.
//

#ifndef ARM_MOTION_MOTIONACTOR_H
#define ARM_MOTION_MOTIONACTOR_H

#include "arm_motion/arm_motion_node.h"
#include <actionlib/server/simple_action_server.h>

class MotionActor
{
    public:
        MotionActor( std::string name ) : action_server( node_handle, name, false ), action_name(name)
        {
            action_server.registerGoalCallback( boost::bind( &MotionActor::goalCallBack, this ) );
            action_server.registerPreemptCallback( boost::bind( &MotionActor::preemptCallBack, this ) );
        }

    protected:
        void goalCallBack()
        {

        }

        void preemptCallBack()
        {

        }

        /*
         * ROS Stuff
         */
        ros::NodeHandle node_handle;
        /* Action Server */
        actionlib::SimpleActionServer<arm_motion::ArmMotionAction> action_server;
        std::string action_name;
        //arm_motion::ArmMotionActionGoal goal;
        arm_motion::ArmMotionActionFeedback feedback;
        arm_motion::ArmMotionActionResult result;
};


#endif //ARM_MOTION_MOTIONACTOR_H
