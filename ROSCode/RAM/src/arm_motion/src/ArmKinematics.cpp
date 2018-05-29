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
    std::vector<Matrix4> DHMatrices;

    /* need to load DH parameters in from LUA */
    double theta[MAX_SERVO] = { 0, 0, 0, 0 };
    double alpha[MAX_SERVO] = { M_PI_2, 0 , 0, 0};
    double r[MAX_SERVO] = { 0, length2, length3, length4 };
    double d[MAX_SERVO] = { length1, 0, 0, 0 };

    DHMatrices.clear();
    for( int i = 0; i < MAX_SERVO; i++ )
    {
        DHMatrices.push_back( HomogenousDHMatrix( _joints.position[i] + theta[i],
                                            alpha[i], r[i], d[i] ) );
    }
    Matrix4 Final = DHMatrices[0];
    for( int i = 1; i < MAX_SERVO; i++ )
    {
        Final *= DHMatrices[i];
    }

    _servo_based_fk.position.x = Final( 0, 3 );
    _servo_based_fk.position.y = Final( 1, 3 );
    _servo_based_fk.position.z = Final( 2, 3 );
    /* ugly, should be handled with a lua config for universality */
    _servo_based_fk.orientation.w = _joints.position[1] + _joints.position[2] + _joints.position[3];
}

void ArmKinematics::publishServoForwardKinematics()
{
    _servo_fk_publisher.publish( _servo_based_fk );
}

Matrix4 ArmKinematics::HomogenousDHMatrix( double theta, double alpha, double r, double d )
{
    Matrix4 h_dh_matrix;

    h_dh_matrix << cos(theta), (-1.0)*sin(theta)*cos(alpha), sin(theta)*sin(alpha), r*cos(theta),
            sin(theta), cos(theta)*cos(alpha), (-1.0)*cos(theta)*sin(alpha), r*sin(theta),
            0, sin(alpha), cos(alpha), d,
            0, 0, 0, 1;

    return h_dh_matrix;
}