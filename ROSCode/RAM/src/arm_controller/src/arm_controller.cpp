//
// Created by korisd on 4/20/18.
//

#include "arm_controller.h"

int main( int argc, char **argv )
{
    ros::init( argc, argv, "arm_controller", ros::init_options::NoSigintHandler );
    ros::NodeHandle ros_handle;

    setupPublishers( ros_handle );
    setupSubscribers( ros_handle );
    setupDynamixelBus();
    setupDynamixelDriver();

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
    servo_rotation_info = ros_handle.advertise<dynamixel_workbench_msgs::XH>( "servo/rotation", 10 );
    servo_shoulder_info = ros_handle.advertise<dynamixel_workbench_msgs::XH>( "servo/shoulder", 10 );
    servo_elbow_info = ros_handle.advertise<dynamixel_workbench_msgs::XH>( "servo/elbow", 10 );
    servo_wrist_info = ros_handle.advertise<dynamixel_workbench_msgs::XH>( "servo/wrist", 10 );
    return;
}

void setupSubscribers( ros::NodeHandle &ros_handle )
{
    enqueue_waypoint = ros_handle.subscribe( "arm/waypoint", 10, enqueueHandler );
    reset_queue = ros_handle.subscribe( "arm/queue_reset", 10, resetQueueHandler );
    operation_mode = ros_handle.subscribe( "arm/mode_setter", 10, operationModeHandler );
    return;
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
    uint8_t servo_ids[5] = { 0, 0, 0, 0, 0 };
    bool success = false;

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
        for( int i = 1; i < 5; i++ )
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
