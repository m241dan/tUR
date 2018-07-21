//
// Created by korisd on 5/29/18.
//

#ifndef ARM_MOTION_TRIALMANAGER_H
#define ARM_MOTION_TRIALMANAGER_H

#include "arm_motion/arm_motion_node.h"
#include "arm_motion/ArmTrial.h"
#include <vector>
#include <memory>

extern "C" {
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
}

class TrialManager
{
    public:
        TrialManager( std::string name );
    protected:
        /*
         * Functions
         */
        void setupSubscribers();
        void setupPublishers();
        void setupTimers();
        void setupLua();

        void enqueueTrial( const std_msgs::UInt8ConstPtr &msg );
        void servoFK( const geometry_msgs::PoseConstPtr &msg );
        void trialMonitor( const ros::TimerEvent &event );
        bool nextTrial();
        void pauseMonitor();
        void resumeMonitor();
        /*
         * Variables
         */
        geometry_msgs::Pose _servo_based_fk;
        /* Ros Specific */
        ros::NodeHandle _node_handle;
        ros::Subscriber trial_selector;
        ros::Subscriber servo_based_fk_subscriber;
        ros::Subscriber manual_waypoint;

        ros::Timer _trial_monitor;

        /* Lua Specific */
        lua_State *_lua_handle;
        std::vector<std::unique_ptr<ArmTrial>> _trial_queue;
};


#endif //ARM_MOTION_TRIALMANAGER_H
