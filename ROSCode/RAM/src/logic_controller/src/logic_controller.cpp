//
// Created by korisd on 5/10/18.
//

#include "logic_controller/logic_controller.h"

int main( int argc, char **argv )
{
    bool run_ros = true;
    ros::init( argc, argv, "logic_controller", ros::init_options::NoSigintHandler );
    ros::NodeHandle ros_handle;

    setupPublishers( ros_handle );
    setupSubscribers( ros_handle );
    setupCallbackFunctions( ros_handle );

    Trial test_trial( "test" );
    geometry_msgs::Pose present_position;
    test_trial.setPresentKinematics( &present_position );
    geometry_msgs::PoseArray test= test_trial.generateWaypoints();
    std::cout << test << std::endl;

    ros::spin();
}

/*
 * Boot Functions
 */
void setupPublishers( ros::NodeHandle &ros_handle )
{
    waypoint_publisher = ros_handle.advertise<geometry_msgs::Pose>( "arm/waypoint", 10 );
    queue_resetter = ros_handle.advertise<std_msgs::UInt8>( "arm/queue_reset", 10 );
}

void setupSubscribers( ros::NodeHandle &ros_handle )
{
    enqueue_trial = ros_handle.subscribe( "logic/trial", 10, enqueueTrialHandler );
    reset_trial_queue = ros_handle.subscribe( "logic/trial_reset", 10, resetTrialHandler );
    queue_size = ros_handle.subscribe( "arm/queue_size", 10, queueSizeHandler );
    present_kinematics = ros_handle.subscribe( "arm/present_kinematics", 10, presentKinematicsHandler );
    goal_kinematics = ros_handle.subscribe( "arm/goal_kinematics", 10, goalKinematicsHandler );
    desired_mode = ros_handle.subscribe( "arm/desired_mode", 10, desiredModeHandler );
    state_machine = ros_handle.subscribe( "arm/state_machine", 10, smStateHandler );
    trial_mode_setter = ros_handle.subscribe( "logic/trial_mode_setter", 10, trialModeHandler );
}

void setupCallbackFunctions( ros::NodeHandle &ros_handle )
{

}

void enqueueTrialHandler( const std_msgs::UInt16::ConstPtr &message )
{

}

void resetTrialHandler( const std_msgs::UInt8::ConstPtr &message )
{

}

void queueSizeHandler( const std_msgs::UInt16::ConstPtr &message )
{

}

void presentKinematicsHandler( const geometry_msgs::Pose::ConstPtr &message )
{

}

void goalKinematicsHandler( const geometry_msgs::Pose::ConstPtr &message )
{

}

void desiredModeHandler( const std_msgs::UInt8::ConstPtr &message )
{

}

void smStateHandler( const std_msgs::String::ConstPtr &message )
{

}

void trialModeHandler( const std_msgs::UInt8::ConstPtr &message )
{

}
