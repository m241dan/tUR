//
// Created by korisd on 5/28/18.
//

#include "arm_motion/arm_motion_node.h"
#include "arm_motion/ArmKinematics.h"

int main( int argc, char **argv )
{
    ros::init( argc, argv, "arm_kinematics" );
    ros::NodeHandle nh;
    ROS_INFO( "Arm Kinematics Node: Boot Starting" );
    ArmKinematics kinematics( "something" );
    ROS_INFO( "Arm Kinematics Node: Boot Complete" );
    ROS_INFO( "Arm Kinematics Node: Spinning with no issues" );
    ros::spin();
    return 0;
}
