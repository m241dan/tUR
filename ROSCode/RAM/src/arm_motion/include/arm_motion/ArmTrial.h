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

typedef std::vector<double> JointPositions;

/*
 * TODO: update these if they change in both places
 */
#define length1 2.6
#define length2 12.6
#define length3 9.01
#define length4 11.00

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
        /*
         * TODO: turn path planning into its own node someday
         */
        geometry_msgs::Pose generateMotionGoalPose( Action a );
        PathConstants generateConstants( geometry_msgs::Pose pose, uint8_t precision );
        void generatePath( std::vector<geometry_msgs::Pose> *path, PathConstants constants, uint8_t precision );
        void buildJointsPosition( std::vector<sensor_msgs::JointState> *goals, std::vector<geometry_msgs::Pose> *path );
        void buildJointsVelocity( std::vector<sensor_msgs::JointState> *goals, uint8_t velocity );
        void buildJointsEffort( std::vector<sensor_msgs::JointState> *goals, uint16_t smoothness, uint16_t tolerance );

        void motionCompleteCallback( const actionlib::SimpleClientGoalState &state, const arm_motion::ArmMotionResultConstPtr &result );
        void motionFeedbackCallback( const arm_motion::ArmMotionActionFeedbackConstPtr &feedback );

        /*
         * TODO: put inverse kinematics into the kinematics node
         */
        JointPositions inverseKinematics( geometry_msgs::Pose pose );

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
