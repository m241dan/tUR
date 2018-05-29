//
// Created by korisd on 5/29/18.
//

#include "arm_motion/ArmTrial.h"

ArmTrial::ArmTrial( std::string trial_name, lua_State *lua, bool &success ) : _trial_name(trial_name)
{
    setupTimers();
}

void ArmTrial::setupTimers()
{
    _run_timer = _node_handle.createTimer( ros::Duration( 0.1 ), boost::bind( &ArmTrial::trialOperation, this, _1 ), false, false );
}

void ArmTrial::trialOperation( const ros::TimerEvent &event )
{

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
    }
    else
    {
        ROS_ERROR( "%s: %s attempting to start an already completed trial", __FUNCTION__, _trial_name.c_str() );
        success = false;
    }
    return success;
}