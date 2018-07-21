//
// Created by korisd on 5/25/18.
//

#include "arm_motion/MotionActor.h"

MotionActor::MotionActor( std::string name, DynamixelController &controller ) :
        action_server( node_handle, name, false ),
        action_name(name),
        _controller(controller),
        joint_goals(4)
{
    action_server.registerGoalCallback( boost::bind( &MotionActor::goalCallBack, this ) );
    action_server.registerPreemptCallback( boost::bind( &MotionActor::preemptCallBack, this ) );
    action_server.start();
    action_timer = node_handle.createTimer( ros::Duration(0.05), boost::bind( &MotionActor::motionMonitor, this, _1 ), false, false );
}

void MotionActor::goalCallBack()
{
    /*setup the new motion*/
    joint_goals = action_server.acceptNewGoal()->joints;
    goal_step = 0;
    goal_max = (uint8_t)joint_goals.size();
    performMotionStep(); //the first, as soon as it receives the goal
    action_timer.start();
}

void MotionActor::preemptCallBack()
{
    result.success = 0;
    action_server.setPreempted( result );
}

void MotionActor::motionMonitor( const ros::TimerEvent &event )
{
    if( action_server.isActive() )
    {
        if( checkMotionStep() ) // check for arrival
        {
            goal_step++;
            if( goal_step < goal_max )
            {
                /* if we haven't completed motion */
                if( !performMotionStep() )
                {
                    result.success = 0;
                    action_server.setAborted( result );
                }
                else
                {
                    feedback.on_step = goal_step;
                    action_server.publishFeedback( feedback );
                }
            }
            else
            {
                /* motion is completed */
                result.success = 1;
                action_server.setSucceeded( result );
            }
        }
    }
    else
    {
        /* we got preempted */
        action_timer.stop();
    }
}

bool MotionActor::checkMotionStep()
{
    bool arrived = true;
    std::vector<int32_t> servo_positions = _controller.getServoPositions();
    std::vector<int32_t> servo_goals = _controller.getServoGoals();

    for( int i = 0; i < MAX_ARM_SERVO; i++ )
    {
        auto tolerance = (int32_t)joint_goals[goal_step].effort[i]; //oddly this works out that we can use the same i for all three
        if( abs( servo_positions[i] - servo_goals[i] ) > tolerance )
        {
            arrived = false;
            break;
        }
    }
    return arrived;
}

bool MotionActor::performMotionStep()
{
    bool success = true;
    if( goal_step != goal_max )
    {
        uint8_t id = 0;
        for( auto velocity : joint_goals[goal_step].velocity )
        {
            id++;
            if( !_controller.changeVelocity( id, (uint32_t )velocity ) )
            {
                success = false;
            }
        }

        id = 0;
        for( auto position : joint_goals[goal_step].position )
        {
            id++;
            if(! _controller.changePosition( id, position ) )
            {
                success = false;
            }
        }
    }
    else
    {
        success = false;
    }
    return success;
}
