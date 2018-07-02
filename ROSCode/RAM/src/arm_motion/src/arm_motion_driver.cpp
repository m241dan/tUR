//
// Created by korisd on 5/25/18.
//

#include <arm_motion/DynamixelController.h>
#include <arm_motion/MotionActor.h>
#include "arm_motion/arm_motion_node.h"

int main( int argc, char **argv )
{
    std::string bus = "/dev/ttyUSB0";
    ros::init( argc, argv, "arm_driver" );
    ros::NodeHandle node_handle;

    ROS_INFO( "Arm Driver Node: Boot Starting" );

    node_handle.param( "bus_name", bus );
    DynamixelController controller( bus );
    MotionActor actor( ros::this_node::getName(), controller );
    ROS_INFO( "Arm Driver Node: Boot Complete" );
    ROS_INFO( "Arm Driver Node: Spinning with no issue" );
    ros::spin();
    return 0;
}