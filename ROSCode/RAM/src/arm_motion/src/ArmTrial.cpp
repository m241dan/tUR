//
// Created by korisd on 5/29/18.
//

#include "arm_motion/ArmTrial.h"

ArmTrial::ArmTrial( std::string trial_name, lua_State *lua, bool &success ) : _trial_name(trial_name),
                                                                              _active(false), _complete(false),
                                                                              _on_action(0)
{
    std::stringstream ss;
    ss << "/home/korisd/tUR/ROSCode/RAM/src/logic_controller/scripts/" << trial_name << ".lua";

    std::cout << "Path: " << ss.str().c_str() << std::endl;
    if( luaL_loadfile( lua, ss.str().c_str() ) || lua_pcall( lua, 0, 1, 0 ) )
    {
        ROS_ERROR( "%s: %s", __FUNCTION__, lua_tostring( lua, -1 ) );
        success = false;
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
        success = true;
        setupSubscribers();
        setupTimers();
    }
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
        _active = true;
    }
    else
    {
        ROS_ERROR( "%s: %s attempting to start an already completed trial", __FUNCTION__, _trial_name.c_str() );
        success = false;
    }
    return success;
}