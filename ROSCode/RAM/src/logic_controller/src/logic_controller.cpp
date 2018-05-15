//
// Created by korisd on 5/10/18.
//

#include "logic_controller/logic_controller.h"

int main( int argc, char **argv )
{
    bool run_ros = true;
    ros::init( argc, argv, "logic_controller", ros::init_options::NoSigintHandler );
    ros::NodeHandle ros_handle;

    setupPublishers( ros_handle );
    setupSubscribers( ros_handle );
    setupCallbackFunctions( ros_handle );

    ros::spin();
}

/*
 * Boot Functions
 */
void setupPublishers( ros::NodeHandle &ros_handle )
{
    waypoint_publisher = ros_handle.advertise<geometry_msgs::Pose>( "arm/waypoint", 10 );
    queue_resetter = ros_handle.advertise<std_msgs::UInt8>( "arm/queue_reset", 10 );
}

void setupSubscribers( ros::NodeHandle &ros_handle )
{
    enqueue_trial = ros_handle.subscribe( "logic/trial", 10, enqueueTrialHandler );
    reset_trial_queue = ros_handle.subscribe( "logic/trial_reset", 10, resetTrialHandler );
    queue_size = ros_handle.subscribe( "arm/queue_size", 10, queueSizeHandler );
    present_kinematics = ros_handle.subscribe( "arm/present_kinematics", 10, presentKinematicsHandler );
    goal_kinematics = ros_handle.subscribe( "arm/goal_kinematics", 10, goalKinematicsHandler );
    desired_mode = ros_handle.subscribe( "arm/desired_mode", 10, desiredModeHandler );
    state_machine = ros_handle.subscribe( "arm/state_machine", 10, smStateHandler );
    trial_mode_setter = ros_handle.subscribe( "logic/trial_mode_setter", 10, trialModeHandler );
}

void setupCallbackFunctions( ros::NodeHandle &ros_handle )
{
    state_machine_timer = ros_handle.createTimer( ros::Duration( 0.1 ), stateMachine );
}

void enqueueTrialHandler( const std_msgs::UInt16::ConstPtr &message )
{

}

void resetTrialHandler( const std_msgs::UInt8::ConstPtr &message )
{

}

void queueSizeHandler( const std_msgs::UInt16::ConstPtr &message )
{

}

void presentKinematicsHandler( const geometry_msgs::Pose::ConstPtr &message )
{

}

void goalKinematicsHandler( const geometry_msgs::Pose::ConstPtr &message )
{

}

void desiredModeHandler( const std_msgs::UInt8::ConstPtr &message )
{

}

void smStateHandler( const std_msgs::String::ConstPtr &message )
{

}

void trialModeHandler( const std_msgs::UInt8::ConstPtr &message )
{

}

/*
 * Timer Functions
 */
void stateMachine( const ros::TimerEvent &event )
{
    forceTransition( getTransition() );
    action();
}

/*
 * State Machine Functiions
 */
void forceTransition( LOGIC_STATE transition_to )
{
    LOGIC_STATE prev_state = state;

    state = transition_to;

    if( state != prev_state )
    {
        /* onExit bits */
        switch( prev_state )
        {
            default: break;
        }

        /* onEnter bits */
        switch( state )
        {
            default: break;
            case MANUAL_STATE:
                /* send reset waypoint queue to arm */
                break;
        }
    }

}

LOGIC_STATE getTransition()
{
    LOGIC_STATE transition_to = state;

    if( inputs.trial_mode == 4 )
    {
        transition_to = MANUAL_STATE;
    }
    else
    {
        switch( state )
        {
            default:
                break;
            case MANUAL_STATE:
                transition_to = WAITING_STATE;
                break;
            case WAITING_STATE:
                if( inputs.trials_queue.size() > 0 )
                {
                    transition_to = LOADING_STATE;
                }
                break;
            case LOADING_STATE:
                transition_to = PERFORM_STATE;
                break;
            case PERFORM_STATE:
                if( inputs.arm_waypoint_queue_size == 0 )
                    transition_to = VERIFY_STATE;
                break;
            case VERIFY_STATE:
                if( inputs.present_trial->isActionComplete() )
                {
                    if( inputs.present_trial->isTrialComplete() )
                    {
                        inputs.present_trial = nullptr;
                        inputs.trials_queue.erase( inputs.trials_queue.begin() );
                        transition_to = WAITING_STATE;
                    }
                    else
                    {
                        transition_to = LOADING_STATE;
                    }
                }
                break;
        }
    }

    return transition_to;
}

void action()
{
    switch( state )
    {
        default: break;
    }
}
