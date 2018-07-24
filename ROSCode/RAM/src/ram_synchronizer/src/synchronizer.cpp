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
    // setup services for calling the processes that are synchtonized
    // setup serial loop counter for scaling the 5 Hz to 1 in the same loop
    // while loop
        // rate 5 Hz
        // filter serial read/writes at 1 Hz (so once every 5 times)
            // service call to do serial loop
        // service call to do i2c loop
        // service call to do servo loop

    ros::init( argc, argv, "arm_synchronizer" );
    ros::NodeHandle node_handle;
    ros::ServiceClient serial_service = node_handle.serviceClient<std_srvs::Trigger>( serial_service_string );
    ros::ServiceClient servo_loop = node_handle.serviceClient<std_srvs::Trigger>( servo_loop_string );
    ros::ServiceClient i2c_loop = node_handle.serviceClient<std_srvs::Trigger>( i2c_loop_string );
    ros::Rate rate(1.0); // 5Hz, rate takes Hertz
    std_srvs::Trigger trigger;

    int serial_filter = 0;

    while( ros::ok() )
    {
        serial_filter++;
        if( serial_filter == 1 )
        {
            if( ros::service::exists( serial_service_string, true ))
            {
                trigger.response.success = false;
                serial_service.call( trigger );
                if( !trigger.response.success )
                {
                    ROS_ERROR( "Synchronizer failed to run Serial Loop" );
                }
            }
        }
        if( serial_filter == 2 )
        {
            if( ros::service::exists( i2c_loop_string, true ) )
            {
                trigger.response.success = false;
                i2c_loop.call( trigger );
                if( !trigger.response.success )
                {
                    ROS_ERROR( "Synchronizer failed to run I2C Loop" );
                }
            }
        }

        if( serial_filter == 4)
        {
            if( ros::service::exists( servo_loop_string, true ) )
            {
                trigger.response.success = false;
                servo_loop.call( trigger );
                if( !trigger.response.success )
                {
                    ROS_ERROR( "Synchronizer failed to run Servo Loop" );
                }
            }
        }

        rate.sleep();
        if( serial_filter >= 5 )
            serial_filter = 0;
    }

    return 0;
}

