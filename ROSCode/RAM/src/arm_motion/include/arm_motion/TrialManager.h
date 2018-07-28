//
// Created by korisd on 5/29/18.
//

#ifndef ARM_MOTION_TRIALMANAGER_H
#define ARM_MOTION_TRIALMANAGER_H

#include "arm_motion/arm_motion_node.h"
#include "arm_motion/ArmTrial.h"
#include <vector>
#include <memory>

enum ManagerState {
    NOMINAL, FAULT, FAULT_RECOVERY
};

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
        void trialMonitor( const std_msgs::UInt8ConstPtr &msg );
        bool nextTrial();

        void manualWaypoint( const arm_motion::ManualWaypointConstPtr &msg );
        void servoIncrement( const arm_motion::ServoChangeConstPtr &msg );
        void servoDecrement( const arm_motion::ServoChangeConstPtr &msg );
        void resetTrialQueue( const std_msgs::UInt8ConstPtr &msg );
        void modeChange( const std_msgs::UInt8ConstPtr &msg );

        /*
         * Variables
         */
        geometry_msgs::Pose _servo_based_fk;
        /* Ros Specific */
        ros::NodeHandle _node_handle;
        ros::Subscriber trial_selector;
        ros::Subscriber servo_based_fk_subscriber;
        ros::Subscriber _manual_waypoint;
        ros::Subscriber _increment_servo;
        ros::Subscriber _decrement_servo;
        ros::Subscriber _reset_trial_queue;
        ros::Subscriber _mode_change;
        ros::Subscriber _trial_monitor;

        ManagerState _state;

        /* Lua Specific */
        lua_State *_lua_handle;
        std::vector<std::unique_ptr<ArmTrial>> _trial_queue;
        ros::Publisher _trial_data_pub;
};


#endif //ARM_MOTION_TRIALMANAGER_H
