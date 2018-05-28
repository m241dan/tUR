//
// Created by korisd on 5/25/18.
//

#include <arm_motion/DynamixelController.h>
#include <arm_motion/MotionActor.h>
#include "arm_motion/arm_motion_node.h"

int main( int argc, char **argv )
{
    std::string bus = "/dev/ttyUSB0";
    ros::init( argc, argv, "arm_motion" );
    ros::NodeHandle node_handle;

    node_handle.param( "bus_name", bus );
    DynamixelController controller( bus );
    MotionActor actor( ros::this_node::getName(), controller );
    ros::spin();
    return 0;
}