//
// Created by korisd on 5/29/18.
//

#ifndef ARM_MOTION_ARMTRIAL_H
#define ARM_MOTION_ARMTRIAL_H

#include "arm_motion/arm_motion_node.h"
#include "arm_motion/ArmMotionAction.h"
#include <arm_motion/MotionMsg.h>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <tuple>
#include <cmath>
#include "arm_motion/PathService.h"

extern "C" {
    #include "lua.h"
    #include "lauxlib.h"
    #include "lualib.h"
}

typedef std::tuple<double,double,double,double> PathConstants;
typedef std::vector<double> JointPositions;
typedef std::vector<geometry_msgs::Pose> Path;

class ArmTrial
{
    public:
        ArmTrial( std::string trial_name, lua_State *lua, geometry_msgs::Pose &pose, bool *success );
        ~ArmTrial() = default;
        bool isActive();
        bool isComplete();
        bool start();

    protected:
        /*
         * Functions
         */
        void setupServiceClient();
        void setupTimers();
        void trialOperation( const ros::TimerEvent &event );
        void servoBasedFK( const geometry_msgs::Pose::ConstPtr &pose );
        void generateMotion();
        void motionCompleteCallback( const actionlib::SimpleClientGoalState &state, const arm_motion::ArmMotionResultConstPtr &result );
        void motionFeedbackCallback( const arm_motion::ArmMotionActionFeedbackConstPtr &feedback );

        /*
         * Variables
         */
        /* ROS Specific */
        ros::NodeHandle _node_handle;
        ros::Subscriber _servo_based_fk_subscriber;
        ros::ServiceClient _path_service;
        ros::Timer _run_timer;
        actionlib::SimpleActionClient<arm_motion::ArmMotionAction> _action_client;

        std::string _trial_name;
        geometry_msgs::Pose &_servo_based_fk_pose;

        bool _active;
        bool _complete;
        std::vector<arm_motion::MotionMsg> _motions;
        unsigned long _on_motion;
};


#endif //ARM_MOTION_ARMTRIAL_H
