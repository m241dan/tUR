//
// Created by korisd on 5/29/18.
//

#include "arm_motion/ArmTrial.h"
ArmTrial::ArmTrial( std::string trial_name, lua_State *lua, bool *success ) : _trial_name(trial_name),
                                                                              _active(false), _complete(false),
                                                                              _on_action(0), _action_client( _node_handle, "arm_motion_driver", true )
{
    //TODO: this needs to be handled via config stuff
    std::stringstream ss;
    ss << "/home/korisd/tUR/ROSCode/RAM/src/arm_motion/scripts/" << trial_name << ".lua";

    std::cout << "Path: " << ss.str().c_str() << std::endl;
    if( luaL_loadfile( lua, ss.str().c_str() ) || lua_pcall( lua, 0, 1, 0 ) )
    {
        ROS_ERROR( "%s: %s", __FUNCTION__, lua_tostring( lua, -1 ) );
        *success = false;
    }
    else
    {
        /* factor to function */
        lua_len( lua, -1 );
        int size = lua_tointeger( lua, -1 );
        lua_pop( lua, 1 );


        for( int i = 1; i < size+1; i++ )
        {
            Action action;
            /* stack: trial table */
            /* get the action at i */
            lua_pushinteger( lua, i );
            lua_gettable( lua, -2 );

            /*
             * TODO: check for nils!!!
             * TODO: FACTOR
             */
            /*
             * Velocity
             */
            /* stack: trial table -> action[i] */
            lua_pushstring( lua, "velocity" );
            /* stack: trial table -> action[i] -> "velocity" */
            lua_gettable( lua, -2 );
            /* stack: trial table -> action[i] -> velocity num */
            action.velocity = (uint8_t )lua_tointeger( lua, -1 );
            lua_pop( lua, 1 );
            /* stack: trial table -> action[i] */

            /*
             * Type
             */
            lua_pushstring( lua, "type" );
            lua_gettable( lua, -2 );
            action.type = (ActionType)lua_tointeger( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * Precision
             */
            lua_pushstring( lua, "precision" );
            lua_gettable( lua, -2 );
            action.precision = (uint8_t)lua_tointeger( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * Smoothness
             */
            lua_pushstring( lua, "smoothness" );
            lua_gettable( lua, -2 );
            action.smoothness = (uint16_t)lua_tointeger( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * Tolerance
             */
            lua_pushstring( lua, "tolerance" );
            lua_gettable( lua, -2 );
            action.tolerance = (uint16_t)lua_tointeger( lua, - 1 );
            lua_pop( lua, 1 );

            /*
             * Shape
             */
            lua_pushstring( lua, "shape" );
            lua_gettable( lua, -2 );
            action.shape = std::string( lua_tostring( lua, -1 ) );
            lua_pop( lua, 1 );

            /*
             * EEO
             */
            lua_pushstring( lua, "eeo" );
            lua_gettable( lua, -2 );
            action.eeo = lua_tonumber( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * X
             */
            lua_pushstring( lua, "x" );
            lua_gettable( lua, -2 );
            action.x = lua_tonumber( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * Y
             */
            lua_pushstring( lua, "y" );
            lua_gettable( lua, -2 );
            action.y = lua_tonumber( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * Z
             */
            lua_pushstring( lua, "z" );
            lua_gettable( lua, -2 );
            action.z = lua_tonumber( lua, -1 );
            lua_pop( lua, 2 ); /* pop the number and the action off */
            /* stack: trial table */
            _actions.push_back( action );
        }
        lua_pop( lua, 1 );
        /* stack: nil */
        setupSubscribers();
        setupTimers();
        *success = _action_client.waitForServer( ros::Duration( 10 ) );
    }
}
ArmTrial::~ArmTrial()
{

}

void ArmTrial::setupSubscribers()
{
    _servo_based_fk_subscriber = _node_handle.subscribe( "kinematics/servo_based_fk", 10, &ArmTrial::servoBasedFK, this );
}
void ArmTrial::setupTimers()
{
    _run_timer = _node_handle.createTimer( ros::Duration( 0.1 ), boost::bind( &ArmTrial::trialOperation, this, _1 ), false, false );
}

void ArmTrial::trialOperation( const ros::TimerEvent &event )
{

}

void ArmTrial::servoBasedFK( const geometry_msgs::Pose::ConstPtr &pose )
{
    _servo_based_fk_pose = *pose;
}

bool ArmTrial::isActive()
{
    return _active;
}

bool ArmTrial::isComplete()
{
    return _complete;
}

bool ArmTrial::start()
{
    bool success = true;
    if( !_complete )
    {
        _run_timer.start();
        generateMotion();
        _active = true;
    }
    else
    {
        ROS_ERROR( "%s: %s attempting to start an already completed trial", __FUNCTION__, _trial_name.c_str() );
        success = false;
    }
    return success;
}

void ArmTrial::generateMotion()
{
    Action &action = _actions[_on_action];

    /*
     * TODO: currently this is all based on servo kinematics only, eventually will be some approximation of both vision and servo kinematics
     */
    /* figure out our final goal based on type of action */
    geometry_msgs::Pose motion_goal = generateMotionGoalPose( action );
    /*
     * generate constants that will be used to determine goal points along the path of the motion
     */
    PathConstants constants = generateConstants( motion_goal, action.precision );


    /* generate the goal points along the path of the motion */
    std::vector<geometry_msgs::Pose> motion_path = generatePath( constants, (double)action.precision );

    /* build the joint_goals to send to the action server based on our goal_points */
    std::vector<sensor_msgs::JointState> joint_goals( MAX_SERVO );
    buildJointsPosition( &joint_goals, &motion_path );
    buildJointsVelocity( &joint_goals, action.velocity );
    buildJointsEffort( &joint_goals, action.smoothness, action.tolerance );

    arm_motion::ArmMotionGoal action_goal;
    action_goal.joints = joint_goals;

    _action_client.sendGoal( action_goal,
                             boost::bind( &ArmTrial::motionCompleteCallback, this, _1, _2 ),
                             actionlib::SimpleActionClient<arm_motion::ArmMotionAction>::SimpleActiveCallback(),
                             actionlib::SimpleActionClient<arm_motion::ArmMotionAction>::SimpleFeedbackCallback() );
                         //TODO: figure out how to bind the feedback    boost::bind( &ArmTrial::motionFeedbackCallback, this, _1 ) );
}

geometry_msgs::Pose ArmTrial::generateMotionGoalPose( Action a )
{
    geometry_msgs::Pose pose;

    pose.orientation.z = a.velocity;
    switch( a.type )
    {
        default: ROS_ERROR( "%s: action contains bad type", __FUNCTION__ ); break;
        case VISION:break;
        case DISCRETE_R:
        {
            pose.position.x = _servo_based_fk_pose.position.x + a.x;
            pose.position.y = _servo_based_fk_pose.position.y + a.y;
            pose.position.z = _servo_based_fk_pose.position.z + a.z;
            pose.orientation.w = _servo_based_fk_pose.orientation.w + a.eeo;
            break;
        }
        case DISCRETE_W:
            pose.position.x = a.x;
            pose.position.y = a.y;
            pose.position.z = a.z;
            pose.orientation.w = a.eeo;
            break;
    }

    return pose;
}

PathConstants ArmTrial::generateConstants( geometry_msgs::Pose pose, uint8_t precision )
{
    double A = 0.;
    double B = 0.;
    double C = 0.;
    double D = 0.;
    double t = (double) precision;



    A = ( pose.position.x - _servo_based_fk_pose.position.x ) / t;
    B = ( pose.position.y - _servo_based_fk_pose.position.y ) / t;
    C = ( pose.position.z - _servo_based_fk_pose.position.y ) / t;
    D = ( pose.orientation.w - _servo_based_fk_pose.orientation.w ) / t;

    return std::make_tuple( A, B, C, D );
}

Path ArmTrial::generatePath( PathConstants constants, uint8_t precision )
{
    Path path;
    const double A = std::get<0>( constants );
    const double B = std::get<1>( constants );
    const double C = std::get<2>( constants );
    const double D = std::get<3>( constants );

    for( int i = 1; i <= precision; i++ )
    {
        double t = (double)i;
        geometry_msgs::Pose motion_pose;
        motion_pose.position.x = _servo_based_fk_pose.position.x + A * t;
        motion_pose.position.y = _servo_based_fk_pose.position.y + B * t;
        motion_pose.position.z = _servo_based_fk_pose.position.z + C * t;
        motion_pose.orientation.w = _servo_based_fk_pose.orientation.w + D * t;
        path.push_back( motion_pose );
    }
    return path;
}

void ArmTrial::buildJointsPosition( std::vector<sensor_msgs::JointState> *goals, std::vector<geometry_msgs::Pose> *path )
{
    /* go through the path */
    for( int i = 0; i < path->size(); i++ )
    {
        /* gives us our joints in radians for path i */
        JointPositions all_joints = inverseKinematics( path->at(i) );
        /* put for each servo its goal for path i */
        for( int j = 0; j < MAX_SERVO; j++ )
        {
            goals->at(j).position.push_back( all_joints[j] );
        }
    }
    for( int i = 0; i < MAX_SERVO; i++ )
    {
        for( int j = 0; j < goals->front().position.size(); j++ )
        {
            ROS_INFO( "Servo[%d]: Position[%f]", i+1, goals->at(i).position[j] );
        }
    }
}

void ArmTrial::buildJointsVelocity( std::vector<sensor_msgs::JointState> *goals, uint8_t velocity )
{
    for( int i = 0; i < goals->size(); i++ )
    {
        for( int j = 0; j < goals->at(i).position.size(); j++ )
        {
            goals->at(i).velocity.push_back( (double)velocity );
        }
    }

}

void ArmTrial::buildJointsEffort( std::vector<sensor_msgs::JointState> *goals, uint16_t smoothness, uint16_t tolerance )
{
    for( int i = 0; i < goals->size(); i++ )
    {
        for( int j = 0; j < ( goals->at(i).position.size()- 1 ); j++ )
        {
            goals->at( i ).effort.push_back( (double)smoothness );
        }
        goals->at(i).effort.push_back( (double)tolerance );
    }
}


void ArmTrial::motionCompleteCallback( const actionlib::SimpleClientGoalState &state,
                                       const arm_motion::ArmMotionResultConstPtr &result )
{
    if( result->success )
    {
        _on_action++;
        if( _on_action >= _actions.size() )
        {
            _complete = true;
        }
        else
        {
            generateMotion();
        }
    }
    else
    {
        ROS_ERROR( "%s: reporting that Motion has failed.", __FUNCTION__ );
        _complete = true;
    }
}

void ArmTrial::motionFeedbackCallback( const arm_motion::ArmMotionActionFeedbackConstPtr &feedback )
{

}

JointPositions ArmTrial::inverseKinematics( geometry_msgs::Pose pose )
{
    /*
    ROS_INFO( "Goal Pose: X[%f] Y[%f] Z[%f] E[%f]", pose.position.x, pose.position.y, pose.position.z, pose.orientation.w );
    double theta_1 = atan2( pose.position.y, pose.position.x );
    pose.position.z -= length1;
    pose.position.x = sqrt( pose.position.x * pose.position.x + pose.position.y * pose.position.y );
    double x_q = pose.position.x - length4*cos(pose.orientation.w);
    double z_q = pose.position.z - length4*sin(pose.orientation.w);
    double CQ = sqrt( x_q * x_q + z_q * z_q );
    double CP = sqrt( pose.position.x * pose.position.x + pose.position.z * pose.position.z );

    double gamma = atan2( pose.position.z, pose.position.x );
    double beta = acos( ( CQ * CQ + CP * CP - length4 * length4 ) / ( 2 * CQ * CP ) );
    ROS_INFO( "CQ[%f] length2[%f] length4[%f]", CQ, length2, length4 );
    ROS_INFO( "acos( %f )", ( CQ * CQ + length2 * length2 - length3 * length3 ) / ( 2 * CQ * length2) );
    double alpha = acos( ( CQ * CQ + length2 * length2 - length3 * length3 ) / ( 2 * CQ * length2) );
    if( boost::math::isnan<double>( gamma ) )
        ROS_INFO( "gamma is nan" );
    if( boost::math::isnan<double>( beta ) )
        ROS_INFO( "beta is nan" );
    if( boost::math::isnan<double>( alpha ) )
        ROS_INFO( "alpha is nan" );

    double theta_2 = alpha + beta + gamma;
    double theta_3 = (-1) * ( M_PI -  acos( ( length2 * length2 + length3 * length3 - CQ * CQ ) / ( 2 * length2 * length3 ) ) );
    double theta_4 = pose.orientation.w - theta_2 - theta_3;
*/
    double x = pose.position.x;
    double y = pose.position.y;
    double z = pose.position.z;
    double w = pose.orientation.w;

    /* new kinematics by hand minus the theta_4 part */
    ROS_INFO( "PK: X[%f] Y[%f] Z[%f] E[%f]", _servo_based_fk_pose.position.x, _servo_based_fk_pose.position.y, _servo_based_fk_pose.position.z, _servo_based_fk_pose.orientation.w );
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

    JointPositions joints;
    joints.push_back( theta_1 );
    joints.push_back( theta_2 );
    joints.push_back( theta_3 );
    joints.push_back( theta_4 );
    for( int i = 0; i < MAX_SERVO; i++ )
    {
        ROS_INFO( "Inverse Kinematics: Joint[%d] Theta[%f]", i+1, joints[i] );
    }
    return joints;
}