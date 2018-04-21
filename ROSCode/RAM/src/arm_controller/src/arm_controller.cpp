//
// Created by korisd on 4/20/18.
//

#include "arm_controller.h"

int main( int argc, char **argv )
{
    ros::init( argc, argv, "arm_controller", ros::init_options::NoSigintHandler );
    ros::NodeHandle ros_handle;

    setupPublishers( ros_handle );
    setupSubscribers( ros_handle );

    /*
     * turn /dev/ttyUSB0 into a var/macro at some point 
     */
    dynamixel::PortHandler *port_handler = dynamixel::PortHandler::getPortHandler( "/dev/ttyUSB0" );

    ros::spin();
    return 0;
}

void setupPublishers( ros::NodeHandle &ros_handle )
{
    return;
}

void setupSubscribers( ros::NodeHandle &ros_handle )
{

    return;
}

