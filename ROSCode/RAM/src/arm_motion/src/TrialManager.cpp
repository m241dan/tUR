//
// Created by korisd on 5/29/18.
//

#include "arm_motion/TrialManager.h"

TrialManager::TrialManager( std::string name )
{
    setupPublishers();
    setupSubscribers();
    setupTimers();
    setupLua();
}

void TrialManager::setupSubscribers()
{
    trial_selector = _node_handle.subscribe( "ram_network_master/trial/selector", 10, &TrialManager::enqueueTrial, this );
    servo_based_fk_subscriber = _node_handle.subscribe( "kinematics/servo_based_fk", 10, &TrialManager::servoFK, this );
    _manual_waypoint = _node_handle.subscribe( "ram_network_master/trial/manual_waypoint", 10, &TrialManager::manualWaypoint, this );
    _increment_servo = _node_handle.subscribe( "ram_network_master/trial/servo_increment", 10, &TrialManager::servoIncrement, this );
    _decrement_servo = _node_handle.subscribe( "ram_network_master/trial/servo_decrement", 10, &TrialManager::servoDecrement, this );
    _reset_trial_queue = _node_handle.subscribe( "ram_network_master/trial/queue_reset", 10, &TrialManager::resetTrialQueue, this );
    _mode_change = _node_handle.subscribe( "ram_network_master/trial/arm_mode", 10, &TrialManager::modeChange, this );

}

void TrialManager::setupPublishers()
{
    //publish trial data here???
}

void TrialManager::setupLua()
{
    _lua_handle = luaL_newstate();
    luaL_openlibs( _lua_handle );
}

void TrialManager::setupTimers()
{
    _trial_monitor = _node_handle.createTimer( ros::Duration( 2 ), boost::bind( &TrialManager::trialMonitor, this, _1 ) );
}

void TrialManager::enqueueTrial( const std_msgs::UInt8ConstPtr &msg )
{
    std::stringstream ss;
    bool success = false;

    ss << "trial" << (int)msg->data;

    _trial_queue.push_back( std::unique_ptr<ArmTrial>( new ArmTrial( ss.str(), _lua_handle, _servo_based_fk, &success ) ) );
    if( success )
    {
        if( _trial_queue.size() == 1 )  //only trial in the queue, start trial and start monitor ( theoretically they should both be paused )
        {
            _trial_queue.front()->start();
            resumeMonitor();     //start regardless, if its already started, this does nothing
        }
    }
    else
    {
        ROS_INFO( "%s: failed to enqueue %s", __FUNCTION__, ss.str().c_str() );
        _trial_queue.pop_back();    //trial on there is a dead trial, so get rid of it
    }
}

void TrialManager::servoFK( const geometry_msgs::PoseConstPtr &msg )
{
    _servo_based_fk = *msg;
}

void TrialManager::trialMonitor( const ros::TimerEvent &event )
{
    if( _trial_queue.size() != 0 )
    {
        bool active = _trial_queue.at(0)->isActive();
        bool complete = _trial_queue.at(0)->isComplete();

        if( complete )
        {
            nextTrial();
        }
        else if( !active )
        {
            if( !_trial_queue.at(0)->start() )   //if it fails to start, go to the next trial
            {
                nextTrial();
            }
        }

    }
    else
    {
        pauseMonitor();
    }
}

bool TrialManager::nextTrial()
{
    //report data here?
    _trial_queue.erase( _trial_queue.begin() );
    return !_trial_queue.empty();
}

void TrialManager::pauseMonitor()
{
    _trial_monitor.stop();
}

void TrialManager::resumeMonitor()
{
    _trial_monitor.start();
}

void TrialManager::manualWaypoint( const arm_motion::ManualWaypointConstPtr &msg )
{

}

void TrialManager::servoIncrement( const arm_motion::ServoChangeConstPtr &msg )
{

}

void TrialManager::servoDecrement( const arm_motion::ServoChangeConstPtr &msg )
{

}

void TrialManager::resetTrialQueue( const std_msgs::UInt8ConstPtr &msg )
{

}

void TrialManager::modeChange( const std_msgs::UInt8ConstPtr &msg )
{

}