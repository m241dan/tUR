//
// Created by korisd on 5/28/18.
//

#include <ros/ros.h>
#include <math.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Pose.h>

ros::Subscriber april_tag_detection;

void aprilHandler( const apriltags_ros::AprilTagDetectionArray::ConstPtr &msg );

int main( int argc, char **argv )
{
    ros::init( argc, argv, "arm_kinematics" );
    ros::NodeHandle node_handle;

    april_tag_detection = node_handle.subscribe( "/apriltags_one", 10, aprilHandler );

    ros::spin();
    return 0;
}

void aprilHandler( const apriltags_ros::AprilTagDetectionArrayConstPtr &msg )
{
    if( msg->detections.size() > 0 )
    {
        geometry_msgs::Pose pose = msg->detections[0].pose.pose;
        double x = pose.orientation.x;
        double y = pose.orientation.y;
        double z = pose.orientation.z;
        double w = pose.orientation.w;

        /*
        double yaw = asin(
                (2 * x * y ) + (2 * z * w));
        double roll = atan2( ( 2 * y * w ) + ( 2 * x * z ), 1 - ( 2 * y * y ) - ( 2 * z * z ) );
        double pitch = atan2( ( 2* x * w ) + ( 2 * y * z ), 1 - ( 2 * x * x ) - ( 2 * z * z ) );
*/

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