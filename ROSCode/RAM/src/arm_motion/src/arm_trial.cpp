//
// Created by korisd on 5/29/18.
//

#include "arm_motion/arm_motion_node.h"
#include "arm_motion/ArmTrial.h"

int main( int argc, char **argv )
{
    ros::init( argc, argv, "arm_trial" );
    ros::NodeHandle node_handle;

    ROS_INFO( "Arm Trial Node: Boot Starting" );
    ROS_INFO( "Arm Trial Node: Boot Complete" );
    ROS_INFO( "Arm Trial Node: Spinning with no issues" );
    ros::spin();
    return 0;
}
