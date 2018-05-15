//
// Created by korisd on 5/10/18.
//

#ifndef LOGIC_CONTROLLER_H
#define LOGIC_CONTROLLER_H

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Pose.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <vector>
#include "state_machine/Error.h"
#include "dynamixel_workbench_msgs/XH.h"
#include "logic_controller/shared_types.h"


/*
 * Publishers
 */
ros::Publisher waypoint_publisher;  /* geometry_msgs Pose */
ros::Publisher queue_resetter;      /* std_msgs UInt8 */
ros::Publisher logic_state_machine_state; /* std_msgs UInt8 */
//ros::Publisher trial_data; /* custom msg... */

/*
 * Subscribers
 */
ros::Subscriber enqueue_trial; /* std_msgs UInt16 */
ros::Subscriber reset_trial_queue; /* std_msgs Uint8 */
ros::Subscriber queue_size; /* std_mgs UInt16 */
ros::Subscriber present_kinematics; /* geometry_msgs Pose */
ros::Subscriber goal_kinematics; /* geometry_msgs Pose */
ros::Subscriber desired_mode; /* std_msgs UInt8 */
ros::Subscriber state_machine; /* std_msgs String */
ros::Subscriber trial_mode_setter; /* std_msgs UInt8 */
//ros::Subscriber busy_box_status; /* custom msg... */

/*
 * Timers
 */
ros::Timer state_machine_timer;

/*
 * State Machine Vars
 */

LOGIC_STATE state;
InputsTable inputs;

/*
 * Boot Functions
 */
void setupPublishers( ros::NodeHandle &ros_handle );
void setupSubscribers( ros::NodeHandle &ros_handle );
void setupCallbackFunctions( ros::NodeHandle &ros_handle );
void setupStateMachine();

/*
 * Subscribers Functions
 */

void enqueueTrialHandler( const std_msgs::UInt16::ConstPtr &message );
void resetTrialHandler( const std_msgs::UInt8::ConstPtr &message );
void queueSizeHandler( const std_msgs::UInt16::ConstPtr &message );
void presentKinematicsHandler( const geometry_msgs::Pose::ConstPtr &message );
void goalKinematicsHandler( const geometry_msgs::Pose::ConstPtr &message );
void desiredModeHandler( const std_msgs::UInt8::ConstPtr &message );
void smStateHandler( const std_msgs::String::ConstPtr &message );
void trialModeHandler( const std_msgs::UInt8::ConstPtr &message );

/*
 * Timer Functions
 */
void stateMachine( const ros::TimerEvent &event );

/*
 * State Machine Functions
 */
void forceTransition( LOGIC_STATE transition_to );
LOGIC_STATE getTransition();
void action();

#endif //LOGIC_CONTROLLER_H
