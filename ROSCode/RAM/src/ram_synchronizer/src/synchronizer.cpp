//
// Created by korisd on 7/27/18.
//

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

int main( int argc, char *argv[] )
{
    ros::init( argc, argv, "arm_synchronizer" );
    ros::NodeHandle node_handle;
    ros::ServiceClient serial_service = node_handle.serviceClient<std_srvs::Trigger>( "/synchronizer/serial" );
    ros::ServiceClient
    // setup services for calling the processes that are synchtonized
    // setup serial loop counter for scaling the 5 Hz to 1 in the same loop
    // while loop
    // rate 5 Hz
        // filter serial read/writes at 1 Hz (so once every 5 times)
            // service call to do serial loop
        // service call to do servo loop
        // service call to do i2c loop
    return 0;
}

