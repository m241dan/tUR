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
    trial_selector = _node_handle.subscribe( "trial/selector", 10, &TrialManager::enqueueTrial, this );
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

    ss << "trial" << msg->data;

    _trial_queue.emplace_back( ss.str(), _lua_handle, success );
    if( success )
    {
        if( _trial_queue.size() == 1 )  //only trial in the queue, start trial and start monitor ( theoretically they should both be paused )
        {
            _trial_queue.front().start();
            resumeMonitor();     //start regardless, if its already started, this does nothing
        }
    }
    else
    {
        ROS_INFO( "%s: failed to enqueue %s", __FUNCTION__, ss.str().c_str() );
        _trial_queue.pop_back();    //trial on there is a dead trial, so get rid of it
    }
}

void TrialManager::trialMonitor( const ros::TimerEvent &event )
{
    if( _trial_queue.size() != 0 )
    {
        bool active = _trial_queue.at(0).isActive();
        bool complete = _trial_queue.at(0).isComplete();

        if( complete )
        {
            nextTrial();
        }
        else if( !active )
        {
            if( !_trial_queue.at(0).start() )   //if it fails to start, go to the next trial
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