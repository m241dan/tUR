//
// Created by korisd on 4/20/18.
//

#include "arm_controller.h"

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
    queue_size = ros_handle.advertise<std_msgs::UInt16>( "arm_queue_size", 10 );
    state_machine_mode = ros_handle.advertise<std_msgs::UInt8>( "state_machine_mode", 10 );
    desired_mode = ros_handle.advertise<std_msgs::UInt8>( "desired_mode", 10 );
    current_kinematics = ros_handle.advertise<geometry_msgs::Pose>( "current_kinematics", 10 );
    goal_kinematics = ros_handle.advertise<geometry_msgs::Pose>( "goal_kinematics", 10 );
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

    if( success )
    {
        for ( int i = ROTATION_SERVO; i < MAX_SERVO; i++)
        {
            /*
             * This is going to be very explicit but slightly slower in the growth department.
             * However, since this doesn't really have to deal with success checking, it should be fine.
             */
            int id = i + 1;
            servos[i].Torque_Enable = (uint8_t)  bench.itemRead(id, "Torque_Enable");
            servos[i].Goal_Position = (uint32_t) bench.itemRead(id, "Goal_Position");
            servos[i].Present_Position = bench.itemRead(id, "Present_Position");
            servos[i].Present_Velocity = bench.itemRead(id, "Present_Velocity");
            servos[i].Profile_Velocity = (uint32_t) bench.itemRead(id, "Profile_Velocity");
        }
    }
    return success;
}

/*
 * Subscriber Callbacks
 */
void enqueueHandler( const geometry_msgs::Pose::ConstPtr &message )
{

}

void resetQueueHandler( const std_msgs::UInt8::ConstPtr &message )
{

}
void operationModeHandler( const std_msgs::UInt8::ConstPtr &message )
{

}

void publishServoInfo( const ros::TimerEvent& event )
{
    for( int i = ROTATION_SERVO; i < MAX_SERVO; i++ )
    {
        servo_info[i].publish( servos[i] );
    }
}