//
// Created by korisd on 4/20/18.
//

#include "arm_controller/arm_controller.h"

/*
 * MAIN FUNCTION
 */
int main( int argc, char **argv )
{
    bool run_ros = true;
    ros::init( argc, argv, "arm_controller", ros::init_options::NoSigintHandler );
    ros::NodeHandle ros_handle;

    setupPublishers( ros_handle );
    setupSubscribers( ros_handle );
    setupCallbackFunctions( ros_handle );
    setupStateMachine();

    if( setupDynamixelBus() )
    {
        if( setupDynamixelDriver() )
        {
            /* handle first read/update and perform an initial reading */
            readAndUpdateServos();
        }
        else
        {
            run_ros = false;
        }
    }
    else
    {
        run_ros = false;
    }


    if( run_ros )
        ros::spin();

    return 0;
}

void setupPublishers( ros::NodeHandle &ros_handle )
{
    queue_size = ros_handle.advertise<std_msgs::UInt16>( "arm/queue_size", 10 );
    state_machine_mode = ros_handle.advertise<std_msgs::String>( "arm/state_machine_mode", 10 );
    desired_mode = ros_handle.advertise<std_msgs::UInt8>( "arm/desired_mode", 10 );
    current_kinematics = ros_handle.advertise<geometry_msgs::Pose>( "arm/current_kinematics", 10 );
    goal_kinematics = ros_handle.advertise<geometry_msgs::Pose>( "arm/goal_kinematics", 10 );
    for( int i = ROTATION_SERVO; i < MAX_SERVO; i++ )
        servo_info[i] = ros_handle.advertise<dynamixel_workbench_msgs::XH>( servo_topic_names[i].c_str(), 10 );
}

void setupSubscribers( ros::NodeHandle &ros_handle )
{
    enqueue_waypoint = ros_handle.subscribe( "arm/waypoint", 10, enqueueHandler );
    reset_queue = ros_handle.subscribe( "arm/queue_reset", 10, resetQueueHandler );
    operation_mode = ros_handle.subscribe( "arm/mode_setter", 10, operationModeHandler );
}

void setupCallbackFunctions( ros::NodeHandle &ros_handle )
{
    publish_info_timer = ros_handle.createTimer( ros::Duration( 1 ), publishServoInfo );
    state_machine_timer = ros_handle.createTimer( ros::Duration( 0.1 ), stateMachineLoop );
}

void setupStateMachine()
{
    machine.addState( off_state.getIdentifier(), &off_state );
    machine.addState( pause_state.getIdentifier(), &pause_state );
    machine.addState( waiting_state.getIdentifier(), &waiting_state );
    machine.addState( go_state.getIdentifier(), &go_state );
    machine.addState( go_synchronized_state.getIdentifier(), &go_synchronized_state );
}

bool setupDynamixelBus()
{
    bool success = false;
    /*
     * turn /dev/ttyUSB0 into a var/macro at some point
     */
    if( bench.begin( "/dev/ttyUSB0" ) )
    {
        messaging::successMsg( __FUNCTION__, "Opened dynamixel servo bus successfully." );
        success = true;
    }
    else
    {
        messaging::errorMsg( __FUNCTION__, "Failed to open dynamixel servo bus." );
    }
    return success;
}

bool setupDynamixelDriver()
{
    uint8_t num_servos;
    uint8_t servo_ids[4] = { 0, 0, 0, 0 };
    bool success = true;

    bench.scan( servo_ids, &num_servos, 4 );

    if( num_servos != 4 )
    {
        /* check for correct count */
        messaging::errorMsg( __FUNCTION__, "Incorrect number of servos found on scan. " );
        success = false;
    }
    else
    {
        /*
         * make sure there are no zero ID servos
         * this is sort of a useless test... but humor me for now
         */
        for( int i = 0; i < 4; i++ )
        {
            if( servo_ids[i] == 0 )
            {
                messaging::errorMsg( __FUNCTION__, "A servo with ID 0 found..." );
                success = false;
            }
        }

    }
    return success;
}

bool readAndUpdateServos()
{
    bool success = true;

    for ( int i = ROTATION_SERVO; i < MAX_SERVO; i++)
    {
        /*
         * This is going to be very explicit but slightly slower in the growth department.
         * However, since this doesn't really have to deal with success checking, it should be fine.
         */
        int id = i + 1;
        inputs.servos[i].Torque_Enable = (uint8_t)  bench.itemRead(id, "Torque_Enable");
        inputs.servos[i].Goal_Position = (uint32_t) bench.itemRead(id, "Goal_Position");
        inputs.servos[i].Present_Position = bench.itemRead(id, "Present_Position");
        inputs.servos[i].Present_Velocity = bench.itemRead(id, "Present_Velocity");
        inputs.servos[i].Profile_Velocity = (uint32_t) bench.itemRead(id, "Profile_Velocity");
        inputs.servos[i].Present_Temperature = (uint8_t) bench.itemRead(id, "Present_Temperature");
        bench.itemWrite( id, "Velocity_Limit", MAX_VELOCITY );
        bench.itemWrite( id, "Position_I_Gain", PID_I_GAIN );
        std::cout << "Servo[" << id << "] Present/Goal[" << inputs.servos[i].Present_Position << "/" << inputs.servos[i].Goal_Position << "]" << std::endl;
    }
    return success;
}

void updateAndPublishCurrentLocation()
{
    /*
     * Update Portion
     */
    std::tuple<kinematics::Coordinates, double> forward_kinematics;
    kinematics::Coordinates coords;
    double end_effector_orientation;

    kinematics::Joints current_joints;
    current_joints._1 = bench.convertValue2Radian( 1, inputs.servos[0].Present_Position );
    current_joints._2 = bench.convertValue2Radian( 2, inputs.servos[1].Present_Position );
    current_joints._3 = bench.convertValue2Radian( 3, inputs.servos[2].Present_Position );
    current_joints._4 = bench.convertValue2Radian( 4, inputs.servos[3].Present_Position ) * (-1); /* have to flip this due to our arm setup */

    forward_kinematics = kinematics::forwardKinematics( current_joints );
    coords = std::get<0>( forward_kinematics );
    end_effector_orientation = std::get<1>( forward_kinematics );

    inputs.current_position.position.x = coords.x;
    inputs.current_position.position.y = coords.y;
    inputs.current_position.position.z = coords.z;

    inputs.current_position.orientation.x = end_effector_orientation;

    /*
     * Publish Portion
     */
    current_kinematics.publish( inputs.current_position );

}


/*
 * Subscriber Callbacks
 */
void enqueueHandler( const geometry_msgs::Pose::ConstPtr &message )
{
    if( inputs.desired_mode != OFF_MODE )
    {
        inputs.waypoint_queue.push_back( *message );
    }
}

void resetQueueHandler( const std_msgs::UInt8::ConstPtr &message )
{
    if( message->data == 1 )
    {
        inputs.waypoint_queue.clear();
    }

}
void operationModeHandler( const std_msgs::UInt8::ConstPtr &message )
{
    inputs.desired_mode = message->data;
}

/*
 * Timer Callbacks
 */
void publishServoInfo( const ros::TimerEvent& event )
{
    for( int i = ROTATION_SERVO; i < MAX_SERVO; i++ )
    {
        servo_info[i].publish( inputs.servos[i] );
    }
}

void publishQueueSize()
{
    std_msgs::UInt16 qs;
    qs.data = inputs.waypoint_queue.size();
    queue_size.publish( qs );
}

void publishStateMachineMode()
{
    std_msgs::String current_state_msg;
    current_state_msg.data = machine.getCurrentIdentifier();
    state_machine_mode.publish( current_state_msg );
}

void publishDesiredMode()
{
    std_msgs::UInt8 mode_message;
    mode_message.data = inputs.desired_mode;
    desired_mode.publish( mode_message );
}

void stateMachineLoop( const ros::TimerEvent& event )
{
    /* read and update servos */
    readAndUpdateServos();
    /* update and publish current location */
    updateAndPublishCurrentLocation();
    /* state machine operation */
    machine.run();
    /* write the new servo goal locations from the output table */
    std::string current_state = machine.getCurrentIdentifier();
    std::vector<ServoCommand> commands;

    if( current_state == OFF_STATE )
    {
        commands = off_state.getOutputs();
    }
    else if( current_state == PAUSE_STATE )
    {
        commands = pause_state.getOutputs();
    }
    else if( current_state == WAITING_STATE )
    {
        commands = waiting_state.getOutputs();
    }
    else if( current_state == GO_STATE )
    {
        commands = go_state.getOutputs();
    }
    else if( current_state == GO_SYNCHRONIZED_STATE )
    {
        commands = go_synchronized_state.getOutputs();
    }

    for( int i = 0; i < commands.size(); i++ )
    {
        bench.itemWrite( commands[i].id, commands[i].command.c_str(), commands[i].value );
    }

    /* publish general data */
    goal_kinematics.publish( inputs.waypoint_queue.front() );

    publishStateMachineMode();
    publishQueueSize();
    publishDesiredMode();
}

