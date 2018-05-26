//
// Created by korisd on 5/25/18.
//

#ifndef ARM_MOTION_DYNAMIXELCONTROLLER_H
#define ARM_MOTION_DYNAMIXELCONTROLLER_H

#include "arm_motion/arm_motion_node.h"
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/XH.h>

class DynamixelController
{
    public:
        DynamixelController( std::string bus );
        bool benchWrite( ServoCommand com );
        int32_t benchRead( uint8_t id, std::string command );

    protected:
        /*
         * functions
         */
        void validateBus();
        void invalidateBus();
        bool verifyServos( int expected_number );
        void initializeServos();
        bool validCommand( ServoCommand com );
        void setupPublishers();
        void setupTimer();
        void updateAndPublishServoInfo( const ros::TimerEvent &event );
        void updateServos();
        void publishServoInfo();

        /*
         * variables
         */
        /*
         * ROS Specific Stuff
         */
        ros::NodeHandle node_handle;
        /* Publishers */
        ros::Publisher servo_info_publishers[MAX_SERVO];
        /* Timers */
        ros::Timer servo_info_timer;

        /*
         * Dynamixel Stuff
         */
        DynamixelWorkbench _bench;
        dynamixel_workbench_msgs::XH servo_info[MAX_SERVO];
        bool _valid;

};


#endif //ARM_MOTION_DYNAMIXELCONTROLLER_H
