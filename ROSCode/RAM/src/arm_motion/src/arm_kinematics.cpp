//
// Created by korisd on 5/28/18.
//

#include "arm_motion/arm_motion_node.h"
#include "arm_motion/ArmKinematics.h"

int main( int argc, char **argv )
{
    ros::init( argc, argv, "arm_kinematics" );

    ROS_INFO( "Arm Kinematics Node: Boot Starting" );
    ArmKinematics kinematics( "something" );
    ROS_INFO( "Arm Kinematics Node: Boot Complete" );
    ROS_INFO( "Arm Kinematics Node: Spinning with no issues" );
    ros::spin();
    return 0;
}

void aprilHandler( const apriltags_ros::AprilTagDetectionArrayConstPtr &msg )
{
    if( !msg->detections.empty() )
    {
        geometry_msgs::Pose pose = msg->detections[0].pose.pose;
        double x = pose.orientation.x;
        double y = pose.orientation.y;
        double z = pose.orientation.z;
        double w = pose.orientation.w;

        /*
         * Copied straight from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
         */
        double roll, pitch, yaw;

        double sinr = 2.0 * (w * x +  y * z );
        double cosr = 1 - 2 * ( x * x + y * y );
        roll = atan2( sinr, cosr);

        double sinp = 2 * ( w* y - z * x);
        if( fabs( sinp) >= 1 )
            pitch = copysign(M_PI / 2, sinp );
        else
            pitch = asin(sinp);

        double siny = 2 * ( w * z + x * y );
        double cosy = 1 - 2 * ( y * y + z * z );
        yaw = atan2( siny, cosy );
        ROS_INFO( "Tag - Roll: %f Pitch: %f Yaw: %f", roll, pitch, yaw );
    }
}