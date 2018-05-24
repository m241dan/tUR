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

    /* init the lua stuff */
    lua_handle = luaL_newstate();
    luaL_openlibs(lua_handle);

    ros::spin();
}

/*
 * Boot Functions
 */
void setupPublishers( ros::NodeHandle &ros_handle )
{
    waypoint_publisher = ros_handle.advertise<geometry_msgs::Pose>( "arm/waypoint", 100 );
    queue_resetter = ros_handle.advertise<std_msgs::UInt8>( "arm/queue_reset", 10 );
    logic_state_machine_state = ros_handle.advertise<std_msgs::UInt8>( "logic/state_machine", 10 );
    repeater_publisher = ros_handle.advertise<std_msgs::UInt16>( "logic/trial", 10 );
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
    repeater_timer = ros_handle.createTimer( ros::Duration( 10 ), repeaterFun );
}

void enqueueTrialHandler( const std_msgs::UInt16::ConstPtr &message )
{
    /* going to write some temporary trial loading code to test things */
    bool success = false;
    std::stringstream ss;
    std::string name;

    ss << "trial" << message->data;

    Trial trial( ss.str(), lua_handle, success );
    trial.setPresentKinematics( &inputs.present_kinematics );
    inputs.trials_queue.push_back( trial );

}

void resetTrialHandler( const std_msgs::UInt8::ConstPtr &message )
{
    if( message->data == 1 )
    {
        inputs.present_trial = nullptr;
        inputs.trials_queue.clear();
        /* reset arm queue */
        state = WAITING_STATE;
    }
}

void queueSizeHandler( const std_msgs::UInt16::ConstPtr &message )
{
    inputs.arm_waypoint_queue_size = message->data;
}

void presentKinematicsHandler( const geometry_msgs::Pose::ConstPtr &message )
{
    inputs.present_kinematics = *message;
}

void goalKinematicsHandler( const geometry_msgs::Pose::ConstPtr &message )
{
    inputs.goal_kinematics = *message;
}

void desiredModeHandler( const std_msgs::UInt8::ConstPtr &message )
{
    inputs.arm_desired_mode = message->data;
}

void smStateHandler( const std_msgs::String::ConstPtr &message )
{
    inputs.arm_state_machine_present_state = message->data;
}

void trialModeHandler( const std_msgs::UInt8::ConstPtr &message )
{
    inputs.trial_mode = message->data;
}

/*
 * Timer Functions
 */
void stateMachine( const ros::TimerEvent &event )
{
    forceTransition( getTransition() );
    action();

    std_msgs::UInt8 present_state;
    present_state.data = state;
    logic_state_machine_state.publish( present_state );
}

void repeaterFun( const ros::TimerEvent &event )
{
    if( inputs.trials_queue.size() == 0 )
    {
        std_msgs::UInt16 trial_num;
        trial_num.data = 1;
        repeater_publisher.publish( trial_num );
    }
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
                /* never load if the arm is "off" */
                if( inputs.trials_queue.size() > 0 && inputs.arm_state_machine_present_state != "OFF_STATE" )
                {
                    transition_to = LOADING_STATE;
                }
                break;
            case LOADING_STATE:
                transition_to = PERFORM_STATE;
                break;
            case PERFORM_STATE:
                if( fabs( ros::Time::now().toSec() - inputs.valid_perform ) > 5 && inputs.arm_waypoint_queue_size == 0 )
                    transition_to = VERIFY_STATE;
                break;
            case VERIFY_STATE:
                if( inputs.present_trial && inputs.present_trial->isActionComplete() ) /* have to check for null here in case the trial gets reset */
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
                else
                {
                    /* trial must have been reset */
                    transition_to = WAITING_STATE;
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
        case MANUAL_STATE:
            /* for now its a do nothing state */
            break;
        case WAITING_STATE:
            /* it's a waiting state, do nothing */
            break;
        case LOADING_STATE:
        {
            if( !inputs.present_trial && inputs.trials_queue.size() > 0 ) /* have to do this check incase trial gets reset */
            {
                inputs.present_trial = &inputs.trials_queue.front();
            }

            if( inputs.present_trial ) /* again, if a trial gets reset */
            {
                geometry_msgs::PoseArray waypoints = inputs.present_trial->generateWaypoints();
                for( int i = 0; i < waypoints.poses.size(); i++ )
                {
                    waypoint_publisher.publish( waypoints.poses.at( i ));
                }
                inputs.valid_perform = ros::Time::now().toSec();
            }
            break;
        }
        case PERFORM_STATE:
            /* do nothing state, its action is the checking in transition */
            break;
        case VERIFY_STATE:
            /* do nothing state, its action occurs in its transition */
            break;
    }
}
