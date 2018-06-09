//
// Created by korisd on 6/5/18.
//

#include "arm_motion/Path.h"

Path::Path( arm_motion::MotionMsg motion, geometry_msgs::Pose pose ) : _motion_guidelines(motion), _present_pose(pose), _A(0.), _B(0.), _C(0.), _D(0.)
{
    generateFinalPose();
    generatePathConstants();
    generatePath();
    pathToJointPositions();            //performs the IK
    setJointPositionVelocities();
    setJointPositionEffort();
}

void Path::generateFinalPose()
{
    _final_pose.orientation.z = _motion_guidelines.velocity;
    switch( _motion_guidelines.type )
    {
        default: ROS_ERROR( "%s: Motion contains bad type", __FUNCTION__ ); break;
        case VISION:break;
        case DISCRETE_R:
        {
            _final_pose.position.x = _present_pose.position.x + _motion_guidelines.x;
            _final_pose.position.y = _present_pose.position.y + _motion_guidelines.y;
            _final_pose.position.z = _present_pose.position.z + _motion_guidelines.z;
            _final_pose.orientation.w = _present_pose.orientation.w + _motion_guidelines.eeo;
            break;
        }
        case DISCRETE_W:
            _final_pose.position.x = _motion_guidelines.x;
            _final_pose.position.y = _motion_guidelines.y;
            _final_pose.position.z = _motion_guidelines.z;
            _final_pose.orientation.w = _motion_guidelines.eeo;
            break;
    }
}

void Path::generatePathConstants()
{
    auto t = (double)_motion_guidelines.precision;

    _A = ( _final_pose.position.x - _present_pose.position.x ) / t;
    _B = ( _final_pose.position.y - _present_pose.position.y ) / t;
    _C = ( _final_pose.position.z - _present_pose.position.y ) / t;
    _D = ( _final_pose.orientation.w - _present_pose.orientation.w ) / t;
}

void Path::generatePath()
{
    for( int i = 1; i <= _motion_guidelines.precision; i++ )
    {
        double t = (double)i;
        geometry_msgs::Pose motion_pose;
        motion_pose.position.x = _present_pose.position.x + _A * t;
        motion_pose.position.y = _present_pose.position.y + _B * t;
        motion_pose.position.z = _present_pose.position.z + _C * t;
        motion_pose.orientation.w = _present_pose.orientation.w + _D * t;
        _motion_path.poses.push_back( motion_pose );
    }
}

void Path::pathToJointPositions()
{

}

void Path::setJointPositionVelocities()
{

}

void Path::setJointPositionEffort()
{

}