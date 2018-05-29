//
// Created by korisd on 5/29/18.
//

#ifndef ARM_MOTION_ARMTRIAL_H
#define ARM_MOTION_ARMTRIAL_H

#include "arm_motion/arm_motion_node.h"

extern "C" {
    #include "lua.h"
    #include "lauxlib.h"
    #include "lualib.h"
}

class ArmTrial
{
    public:
        ArmTrial( std::string trial_name, lua_State *lua, bool &success );
        bool isActive();
        bool isComplete();
        bool start();
    protected:
        /*
         * Functions
         */
        void setupTimers();
        void trialOperation( const ros::TimerEvent &event );
        /*
         * Variables
         */
        /* ROS Specific */
        ros::NodeHandle _node_handle;
        ros::Timer _run_timer;

        std::string _trial_name;
        bool _active;
        bool _complete;
};


#endif //ARM_MOTION_ARMTRIAL_H
