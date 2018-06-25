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

Matrix4 ArmKinematics::HomogenousDHMatrix( double theta, double alpha, double r, double d )
{
    Matrix4 h_dh_matrix;

    h_dh_matrix <<  cos(theta),    -sin(theta)*cos(alpha),          sin(theta)*sin(alpha),  r*cos(theta),
                    sin(theta),     cos(theta)*cos(alpha),         -cos(theta)*sin(alpha),  r*sin(theta),
                    0,              sin(alpha),                     cos(alpha),             d,
                    0,              0,                              0,                      1;

    return h_dh_matrix;
}