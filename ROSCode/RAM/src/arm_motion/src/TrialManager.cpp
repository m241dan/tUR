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
  //  trial_selector = _node_handle.subscribe( "trial/selector", 10,  );
}

void TrialManager::setupPublishers()
{

}

void TrialManager::setupLua()
{
    _lua_handle = luaL_newstate();
    luaL_openlibs( _lua_handle );
}

void TrialManager::setupTimers()
{
    _trial_monitor = _node_handle.createTimer( ros::Duration( 5 ), boost::bind( &TrialManager::trialMonitor, this, _1 ) );
}

void TrialManager::enqueueTrial( const std_msgs::UInt8ConstPtr &msg )
{
    std::stringstream ss;
    bool success = false;

    ss << "trial" << msg->data;

    _trial_queue.emplace_back( ss.str(), _lua_handle, success );
    if( success )
    {
        _trial_monitor.start();     //start regardless, if its already started, this does nothing
    }
    else
    {
        ROS_INFO( "%s: failed to enqueue %s", __FUNCTION__, ss.str() );
        _trial_queue.pop_back();    //trial on there is a dead trial, so get rid of it
    }
}

void TrialManager::trialMonitor( const ros::TimerEvent &event )
{

}