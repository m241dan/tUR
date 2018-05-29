//
// Created by korisd on 5/29/18.
//

#ifndef ARM_MOTION_ARMTRIAL_H
#define ARM_MOTION_ARMTRIAL_H

#include "arm_motion/arm_motion_node.h"
#include "arm_motion/ArmMotionAction.h"
#include "arm_motion/Action.h"
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <utility>

extern "C" {
    #include "lua.h"
    #include "lauxlib.h"
    #include "lualib.h"
}

class ArmTrial
{
    public:
        ArmTrial( std::string trial_name, lua_State *lua, bool &success );
        ArmTrial( ArmTrial&& other ) : _trial_name(std::move(other._trial_name)),
                                       _active(other._active),
                                       _complete(other._complete),
                                       _on_action(other._on_action),
                                       _action_client(std::move(other._action_client))

        {

        }
        ~ArmTrial();
        bool isActive();
        bool isComplete();
        bool start();
        ArmTrial& operator=(const ArmTrial& other) = default;

    protected:
        /*
         * Functions
         */
        void setupSubscribers();
        void setupTimers();
        void trialOperation( const ros::TimerEvent &event );
        void servoBasedFK( const geometry_msgs::Pose::ConstPtr &pose );
        /*
         * Variables
         */
        /* ROS Specific */
        ros::NodeHandle _node_handle;
        ros::Subscriber _servo_based_fk_subscriber;
        ros::Timer _run_timer;
        actionlib::SimpleActionClient<arm_motion::ArmMotionAction> _action_client;

        std::string _trial_name;
        geometry_msgs::Pose _servo_based_fk_pose;

        bool _active;
        bool _complete;
        std::vector<Action> _actions;
        unsigned long _on_action;
};


#endif //ARM_MOTION_ARMTRIAL_H
