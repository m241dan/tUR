//
// Created by korisd on 6/5/18.
//

#include "arm_motion/Path.h"

Path::Path( arm_motion::MotionMsg motion, geometry_msgs::Pose pose ) : _motion_guidelines(motion), _present_pose(pose), _A(0.), _B(0.), _C(0.), _D(0.)
{
    ROS_INFO( "_motion_guidelines.x[%f]", _motion_guidelines.x );
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
            ROS_INFO( "_final_pose_x[%f]", _final_pose.position.x );
            _final_pose.position.y = _motion_guidelines.y;
            _final_pose.position.z = _motion_guidelines.z;
            _final_pose.orientation.w = _motion_guidelines.eeo;
            break;
    }
}

void Path::generatePathConstants()
{
    auto t = (double)_motion_guidelines.precision;
    ROS_INFO( "t[%f]",t );
    ROS_INFO( "fp[%f] pp[%f]", _final_pose.position.x, _present_pose.position.x );
    _A = ( _final_pose.position.x - _present_pose.position.x ) / t;
    _B = ( _final_pose.position.y - _present_pose.position.y ) / t;
    _C = ( _final_pose.position.z - _present_pose.position.y ) / t;
    _D = ( _final_pose.orientation.w - _present_pose.orientation.w ) / t;
    ROS_INFO( "A[%f]", _A );
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
    for( auto pose : _motion_path.poses )
    {
        bool valid = true;
        uint8_t count = 0;
        sensor_msgs::JointState joints;
        /*
         * Believe, in good faith, that we can reach the position we're going to.
         * If we can't, it's invalid. So, we'll loop again, but before we do, we have to change our end effector orientation(ie, orientation.w)
         * If we're believe 20cm, rotate our goal down by M_PI/8. If we're above, rotate it up. And then, run it again with the new EEO
         * Also, only try 4 times before giving up on the position and defaulting a known safe position
         */
        do
        {
            valid = true;
            joints = inverseKinematics( pose );
            for( auto pos : joints.position )
            {
                if( std::isnan( pos ) )
                {
                    valid = false;
                    if( pose.position.z < 20.0 )
                    {
                        pose.orientation.w -= M_PI/8;
                    }
                    else
                    {
                        pose.orientation.w += M_PI/8;
                    }
                    break;
                }
            }
            if( count++ > 4 )
            {
                pose.position.x = 6.0;
                pose.position.y = 0.0;
                pose.position.z = 17.0;
                pose.orientation.w = 0.0;
            }
        } while( !valid );

        //verify

        if( valid )
                _output_joint_states.push_back( joints );
    }
}

void Path::setJointPositionVelocities()
{
    for( auto &joint : _output_joint_states )
    {
        double max_travel = *std::max_element( joint.position.begin(), joint.position.end());
        for( auto position : joint.position )
        {
            double velocity_mutiplier = position / max_travel;
            //This ensures that we always have a joint velocity of at least 2. Which is really slow, so plenty of time to interrupt
            double velocity = 2.0 + fabs( (double)_motion_guidelines.velocity * velocity_mutiplier );
            joint.velocity.push_back( velocity );
        }
    }
}

void Path::setJointPositionEffort()
{
    for( auto &joint : _output_joint_states )
    {
        ROS_INFO( "Smoothness[%d]", (int)_motion_guidelines.smoothness );
        joint.effort = std::vector<double>( joint.position.size(), (double)_motion_guidelines.smoothness );
    }
    auto &last_effort = _output_joint_states.back().effort;
    for( auto &effort : last_effort )
    {
        effort = (double)_motion_guidelines.tolerance;
    }
}

std::vector<sensor_msgs::JointState> &Path::getPath()
{
    return _output_joint_states;
}

sensor_msgs::JointState Path::inverseKinematics( geometry_msgs::Pose &pose )
{
    double x = pose.position.x;
    double y = pose.position.y;
    double z = pose.position.z;
    double w = pose.orientation.w;

    /* new kinematics by hand minus the theta_4 part */
    ROS_INFO( "PK: X[%f] Y[%f] Z[%f] E[%f]", _present_pose.position.x, _present_pose.position.y, _present_pose.position.z, _present_pose.orientation.w );
    ROS_INFO( "X[%f] Y[%f] Z[%f] E[%f]", x, y, z, w );
    double X_new = sqrt( x * x + y * y );
    ROS_INFO( "X_new: %f", X_new );

    double theta_1 = atan2( y, x );
    ROS_INFO( "Theta_1: %f", theta_1 );

    double X_c = X_new - length4*cos(w);
    ROS_INFO( "X_c: %f", X_c );

    double Z_c = z + length4*sin((-1)*w);
    ROS_INFO( "Z_c: %f", Z_c );

    double Z_l = Z_c - length1;
    ROS_INFO( "Z_l: %f", Z_l );

    double length5 = sqrt( X_c * X_c + Z_l * Z_l );
    ROS_INFO( "Length5: %f", length5 );

    double alpha = acos( (length5 * length5 - length2 * length2 - length3 * length3 ) / ( (-2) * length2 * length3 ) );
    ROS_INFO( "Alpha: %f", alpha );

    double theta_3 = (-1) * (M_PI - alpha);
    ROS_INFO( "Theta_3: %f", theta_3 );

    double Z_new = z - length1;
    ROS_INFO( "Z_new: %f", Z_new );

    double alpha_4 = atan2( Z_new, X_new );
    ROS_INFO( "Alpha_4: %f", alpha_4 );

    double length6 = sqrt( Z_new * Z_new + X_new * X_new );
    ROS_INFO( "Length6: %f", length6 );

    double alpha_3 = acos( ( length4 * length4 - length5 * length5 - length6 * length6 ) / ( (-2) * length5 * length6 ) );
    ROS_INFO( "Alpha_3: %f", alpha_3 );

    double alpha_2 = acos( ( length3 * length3 - length5 * length5 - length2 * length2 ) / ( (-2) * length5 * length2 ) );
    ROS_INFO( "Alpha_2: %f", alpha_2 );

    double theta_2 = alpha_2 + alpha_3 + alpha_4;
    ROS_INFO( "Theta_2: %f", theta_2 );

    double theta_4 = w - theta_2 - theta_3;
    ROS_INFO( "Theta_4: %f", theta_4 );

    sensor_msgs::JointState position;
    position.position.push_back( theta_1 );
    position.position.push_back( theta_2 );
    position.position.push_back( theta_3 );
    position.position.push_back( theta_4 );

    return position;
}