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
#include <tuple>

extern "C" {
    #include "lua.h"
    #include "lauxlib.h"
    #include "lualib.h"
}

typedef std::tuple<double,double,double,double> PathConstants;
#define MAKE_PATH_CONSTANTS( A, B, C, D ) std::make_tuple( A, B, C, D )

class ArmTrial
{
    public:
        ArmTrial( std::string trial_name, lua_State *lua, bool *success );
        ~ArmTrial();
        bool isActive();
        bool isComplete();
        bool start();

    protected:
        /*
         * Functions
         */
        void setupSubscribers();
        void setupTimers();
        void trialOperation( const ros::TimerEvent &event );
        void servoBasedFK( const geometry_msgs::Pose::ConstPtr &pose );
        void generateMotion();
        geometry_msgs::Pose generateMotionGoalPose( Action a );
        PathConstants generateConstants( geometry_msgs::Pose pose, uint8_t precision );
        void generatePath( std::vector<geometry_msgs::Pose> *path, PathConstants constants, uint8_t precision );
        void buildJointStates( std::vector<sensor_msgs::JointState> *goals, std::vector<geometry_msgs::Pose> *path );

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
