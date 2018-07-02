//
// Created by korisd on 5/28/18.
//

#include "arm_motion/ArmKinematics.h"

ArmKinematics::ArmKinematics( std::string test )
{
    setupSubscribers();
    setupPublishers();
}

void ArmKinematics::setupSubscribers()
{
    _joints_sub = _node_handle.subscribe( "kinematics/joints_in_radians", 10, &ArmKinematics::servoInfoHandler, this );
    _camera_one_tags = _node_handle.subscribe( "apriltag_detections_one", 10, &ArmKinematics::camOneTagHandler, this );
}

void ArmKinematics::setupPublishers()
{
    _servo_fk_publisher = _node_handle.advertise<geometry_msgs::Pose> ( "kinematics/servo_based_fk", 10 );
}

void ArmKinematics::servoInfoHandler( const sensor_msgs::JointState::ConstPtr &joints )
{
    _joints = *joints;
    updateServoForwardKinematics();
    publishServoForwardKinematics();
}

void ArmKinematics::camOneTagHandler( const apriltags_ros::AprilTagDetectionArrayConstPtr &tags )
{
    _tags_seen = *tags;
    std::cout << _tags_seen << std::endl;
    updateVisionForwardKinematics();
    publishVisionForwardKinematics();
}

void ArmKinematics::updateServoForwardKinematics()
{

    Matrix4 H0_1    = HomogenousDHMatrix( _joints.position[0], M_PI_2, 0, length1 );
    Matrix4 H1_2    = HomogenousDHMatrix( _joints.position[1], 0, length2, 0 );
    Matrix4 H2_3    = HomogenousDHMatrix( _joints.position[2], 0, length3, 0 );
    Matrix4 H3_4    = HomogenousDHMatrix( _joints.position[3], 0, length4, 0 );
    Matrix4 Final   = H0_1 * H1_2 * H2_3 * H3_4;

    _servo_based_fk.position.x = Final( 0, 3 );
    _servo_based_fk.position.y = Final( 1, 3 );
    _servo_based_fk.position.z = Final( 2, 3 );
    _servo_based_fk.orientation.w = _joints.position[1] + _joints.position[2] + _joints.position[3];
}

void ArmKinematics::publishServoForwardKinematics()
{
    _servo_fk_publisher.publish( _servo_based_fk );
}

void ArmKinematics::updateVisionForwardKinematics()
{

    const double z_dist = 0.0349;
    const double y_dist = 0.0222;
    const double x_dist = 0.0667;
    const double h_dist = sqrt( y_dist * y_dist + z_dist * z_dist );
    /*
     * These will eventually be used when the cameras are in a set position, for now, meh
     */
     double yaw_offset = 0.0;
     double roll_offset = 0.0;
     double pitch_offset = 0.0;


    convertTagsQuaternionToRPY();

    geometry_msgs::Pose pose;
    bool found = false;
    for( auto tag : _tags_seen.detections )
    {
        if( tag.id == 8 )
        {
            pose = tag.pose.pose;
            found = true;
            break;
        }
    }

    if( found )
    {
        /*
         * Remember this is wonky because april tag detections are on a different coordinate system
         */
        double y_roll = h_dist * cos( pose.orientation.z );
        double z_roll = h_dist * sin( 45 - pose.orientation.z );

        double z_theta = x_dist * sin( pose.orientation.x );
        double x_theta = x_dist * cos( pose.orientation.x );

        double y_yaw = x_theta * sin( pose.orientation.y );
        double x_yaw = x_theta * cos( pose.orientation.y );


    }

}

void ArmKinematics::publishVisionForwardKinematics()
{

}

void ArmKinematics::convertTagsQuaternionToRPY()
{
    if( !_tags_seen.detections.empty())
    {
        for( auto &tag : _tags_seen.detections )
        {
            geometry_msgs::Pose &pose = tag.pose.pose;
            const double x = pose.orientation.x;
            const double y = pose.orientation.y;
            const double z = pose.orientation.z;
            const double w = pose.orientation.w;

            /*
             * Copied straight from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
             */
            double roll, pitch, yaw;

            double sinr = 2.0 * (w * x + y * z);
            double cosr = 1 - 2 * (x * x + y * y);
            roll = atan2( sinr, cosr );

            double sinp = 2 * (w * y - z * x);
            if( fabs( sinp ) >= 1 )
                pitch = copysign( M_PI / 2, sinp );
            else
                pitch = asin( sinp );

            double siny = 2 * (w * z + x * y);
            double cosy = 1 - 2 * (y * y + z * z);
            yaw = atan2( siny, cosy );

            if( roll < 0 )
            {
                roll += M_PI;
                roll *= (-1);
            }
            else if( roll > 0 )
                roll = M_PI - roll;

            pose.orientation.x = roll;
            pose.orientation.y = pitch;
            pose.orientation.z = yaw;
            pose.orientation.w = 0.0f;
            ROS_INFO( "Tag - Roll: %f Pitch: %f Yaw: %f", tag.pose.pose.orientation.x,
                                                          tag.pose.pose.orientation.y,
                                                          tag.pose.pose.orientation.z );

        }



    }
}
Matrix4 ArmKinematics::HomogenousDHMatrix( double theta, double alpha, double r, double d )
{
    Matrix4 h_dh_matrix;

    h_dh_matrix <<  cos(theta),    -sin(theta)*cos(alpha),          sin(theta)*sin(alpha),  r*cos(theta),
                    sin(theta),     cos(theta)*cos(alpha),         -cos(theta)*sin(alpha),  r*sin(theta),
                    0,              sin(alpha),                     cos(alpha),             d,
                    0,              0,                              0,                      1;

    return h_dh_matrix;
}