//
// Created by korisd on 6/8/18.
//

#include "arm_motion/arm_motion_node.h"
#include "arm_motion/PathPlanner.h"

int main( int argc, char **argv )
{
    ros::init( argc, argv, "arm_path" );

    ROS_INFO( "Arm Path Node: Boot Starting" );
    PathPlanner planner( "myplanner" );
    ROS_INFO( "Arm Path Node: Boot Complete" );
    ROS_INFO( "Arm Path Node: Spinning with no issues" );
    ros::spin();

    return 0;
}