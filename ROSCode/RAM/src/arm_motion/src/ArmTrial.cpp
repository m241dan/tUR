//
// Created by korisd on 5/29/18.
//

#include "arm_motion/ArmTrial.h"


ArmTrial::ArmTrial( std::string trial_name, lua_State *lua, bool *success ) : _trial_name(trial_name),
                                                                              _active(false), _complete(false),
                                                                              _on_motion(), _action_client( _node_handle, "arm_motion_driver", true )
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
        int size = (int)lua_tointeger( lua, -1 );
        lua_pop( lua, 1 );


        for( int i = 1; i < size+1; i++ )
        {
            arm_motion::MotionMsg motion;
            /* stack: trial table */
            /* get the motion at i */
            lua_pushinteger( lua, i );
            lua_gettable( lua, -2 );

            /*
             * TODO: check for nils!!!
             * TODO: FACTOR
             */
            /*
             * Velocity
             */
            /* stack: trial table -> motion[i] */
            lua_pushstring( lua, "velocity" );
            /* stack: trial table -> motion[i] -> "velocity" */
            lua_gettable( lua, -2 );
            /* stack: trial table -> motion[i] -> velocity num */
            motion.velocity = (uint8_t )lua_tointeger( lua, -1 );
            lua_pop( lua, 1 );
            /* stack: trial table -> motion[i] */

            /*
             * Type
             */
            lua_pushstring( lua, "type" );
            lua_gettable( lua, -2 );
            motion.type = (uint8_t)lua_tointeger( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * Precision
             */
            lua_pushstring( lua, "precision" );
            lua_gettable( lua, -2 );
            motion.precision = (uint8_t)lua_tointeger( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * Smoothness
             */
            lua_pushstring( lua, "smoothness" );
            lua_gettable( lua, -2 );
            motion.smoothness = (uint16_t)lua_tointeger( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * Tolerance
             */
            lua_pushstring( lua, "tolerance" );
            lua_gettable( lua, -2 );
            motion.tolerance = (uint16_t)lua_tointeger( lua, - 1 );
            lua_pop( lua, 1 );

            /*
             * Shape
             */
            lua_pushstring( lua, "shape" );
            lua_gettable( lua, -2 );
            motion.shape = std::string( lua_tostring( lua, -1 ) );
            lua_pop( lua, 1 );

            /*
             * EEO
             */
            lua_pushstring( lua, "eeo" );
            lua_gettable( lua, -2 );
            motion.eeo = lua_tonumber( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * X
             */
            lua_pushstring( lua, "x" );
            lua_gettable( lua, -2 );
            motion.x = lua_tonumber( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * Y
             */
            lua_pushstring( lua, "y" );
            lua_gettable( lua, -2 );
            motion.y = lua_tonumber( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * Z
             */
            lua_pushstring( lua, "z" );
            lua_gettable( lua, -2 );
            motion.z = lua_tonumber( lua, -1 );
            lua_pop( lua, 2 ); /* pop the number and the motion off */
            /* stack: trial table */
            _motions.push_back( motion );
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
    _path_service = _node_handle.serviceClient<arm_motion::PathService>("plan_a_path");
}
void ArmTrial::setupTimers()
{
    _run_timer = _node_handle.createTimer( ros::Duration( 0.1 ), boost::bind( &ArmTrial::trialOperation, this, _1 ), false, false );
}

void ArmTrial::trialOperation( const ros::TimerEvent &event )
{
    // TODO: this should probably be removed???
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
    arm_motion::PathService srv;

    srv.request.motion = _motions[_on_motion];
    if( _path_service.call( srv ) )
    {
        arm_motion::ArmMotionGoal action_goal;
        action_goal.joints = srv.response.joints;
        _action_client.sendGoal( action_goal,
                                 boost::bind( &ArmTrial::motionCompleteCallback, this, _1, _2 ),
                                 actionlib::SimpleActionClient<arm_motion::ArmMotionAction>::SimpleActiveCallback(),
                                 actionlib::SimpleActionClient<arm_motion::ArmMotionAction>::SimpleFeedbackCallback() );
        //TODO: figure out how to bind the feedback    boost::bind( &ArmTrial::motionFeedbackCallback, this, _1 ) );

    }
    else
    {
        ROS_ERROR( "%s: cannot perform motion", __FUNCTION__ );
    }
}

void ArmTrial::motionCompleteCallback( const actionlib::SimpleClientGoalState &state,
                                       const arm_motion::ArmMotionResultConstPtr &result )
{
    if( result->success )
    {
        _on_motion++;
        if( _on_motion >= _motions.size() )
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