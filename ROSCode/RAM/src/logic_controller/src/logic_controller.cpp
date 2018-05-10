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
}

void setupCallbackFunctions( ros::NodeHandle &ros_handle )
{

}