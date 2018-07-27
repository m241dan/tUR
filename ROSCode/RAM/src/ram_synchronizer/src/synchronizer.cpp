//
// Created by korisd on 7/27/18.
//

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

const std::string serial_service_string( "/synchronizer/serial" );
const std::string servo_loop_string( "/synchronizer/servos" );
const std::string i2c_loop_string( "/synchronizer/i2c" );

int main( int argc, char *argv[] )
{
    ros::init( argc, argv, "arm_synchronizer" );
    ros::NodeHandle node_handle;
    ros::ServiceClient serial_service = node_handle.serviceClient<std_srvs::Trigger>( "/synchronizer/serial" );
    ros::ServiceClient
    // setup services for calling the processes that are synchtonized
    // setup serial loop counter for scaling the 5 Hz to 1 in the same loop
    // while loop
        // rate 5 Hz
        // filter serial read/writes at 1 Hz (so once every 5 times)
            // service call to do serial loop
        // service call to do servo loop
        // service call to do i2c loop

    ros::init( argc, argv, "arm_synchronizer" );
    ros::NodeHandle node_handle;
    ros::ServiceClient serial_service = node_handle.serviceClient<std_srvs::Trigger>( serial_service_string );
    ros::ServiceClient servo_loop = node_handle.serviceClient<std_srvs::Trigger>( servo_loop_string );
    ros::ServiceClient i2c_loop = node_handle.serviceClient<std_srvs::Trigger>( i2c_loop_string );
    ros::Rate rate(5); // 5Hz, rate takes Hertz
    std_srvs::Trigger trigger;

    uint8_t serial_filter = 0;

    while( ros::ok() )
    {
        if( ++serial_filter >= 5 )
        {
            if( ros::service::exists( serial_service_string, true ))
            {
                serial_service.call( trigger );
                if( !trigger.response.success )
                {
                    ROS_ERROR( "Synchronizer failed to run Serial Loop" );
                }
            }
            serial_filter = 0;
        }

        if( ros::service::exists( servo_loop_string, true ) )
        {
            servo_loop.call( trigger );
            if( !trigger.response.success )
            {
                ROS_ERROR( "Synchronizer failed to run Servo Loop" );
            }
        }

        if( ros::service::exists( i2c_loop_string, true ) )
        {
            i2c_loop.call( trigger );
            if( !trigger.response.success )
            {
                ROS_ERROR( "Synchronizer failed to run I2C Loop" );
            }
        }
        rate.sleep();
    }

    return 0;
}

