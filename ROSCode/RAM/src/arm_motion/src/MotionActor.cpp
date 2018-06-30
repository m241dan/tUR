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
        ROS_INFO( "%s: is active", __FUNCTION__ );
        if( checkMotionStep() ) // check for arrival
        {
            goal_step++;
            ROS_INFO( "%s: goal_step[%d] goal_max[%d]", __FUNCTION__, goal_step, goal_max );
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
        ROS_INFO( "%s: checking tolerance: %d ", __FUNCTION__, (int)tolerance );
        ROS_INFO( "%s: Servo[%d]: Current Position [%d] Goal Position [%d]", __FUNCTION__, i, servo_positions[i], servo_goals[i] );
        if( abs( servo_positions[i] - servo_goals[i] ) > tolerance )
        {
            ROS_INFO( "%s: NOT ARRIVED!!!!!!", __FUNCTION__ );
            arrived = false;
            break;
        }
    }
    return arrived;
}

bool MotionActor::performMotionStep()
{
    ROS_INFO( "performing motion step" );
    bool success = true;
    if( goal_step != goal_max )
    {
        ROS_INFO( "performing loop" );
        uint8_t id = 0;
        for( auto velocity : joint_goals[goal_step].velocity )
        {
            id++;
            ROS_INFO( "%s: attempting to write id[%d] and velocity[%d]", __FUNCTION__, (int)id, (int)velocity );
            if( !_controller.changeVelocity( id, (uint32_t )velocity ) )
            {
                ROS_ERROR( "%s: failed to write profile velocity[%d] to servo[%d]", __FUNCTION__, (int)velocity, (int)id );
                success = false;
            }
        }

        id = 0;
        for( auto position : joint_goals[goal_step].position )
        {
            id++;
            ROS_INFO( "%s: attempting to write id[%d] and position[%f]", __FUNCTION__, (int)id, position );
            if(! _controller.changePosition( id, position ) )
            {
                ROS_ERROR( "%s: failed to write goal position[%f] to servo[%d]", __FUNCTION__, position, (int)id );
                success = false;
            }
        }
    }
    else
    {
        ROS_ERROR( "%s: attempting to perform a goal[%d}] outside the bounds of the goal vectors.", __FUNCTION__, goal_step );
        success = false;
    }
    return success;
}
