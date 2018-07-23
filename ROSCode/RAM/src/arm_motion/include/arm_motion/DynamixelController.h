//
// Created by korisd on 5/25/18.
//

#ifndef ARM_MOTION_DYNAMIXELCONTROLLER_H
#define ARM_MOTION_DYNAMIXELCONTROLLER_H

#include "arm_motion/arm_motion_node.h"
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <vector>

class DynamixelController
{
    public:
        DynamixelController( std::string bus );
        bool benchWrite( ServoCommand com );
        int32_t benchRead( uint8_t id, std::string command );
        std::vector<int32_t> &getServoPositions();
        std::vector<int32_t> &getServoGoals();
        bool torqueOn();
        bool torqueOff();
        bool holdPosition();
        bool changePosition( uint8_t id, int32_t position );
        bool changePosition( uint8_t id, double radian );
        bool changeVelocity( uint8_t id, uint32_t velocity );
        bool loadDefault( uint8_t id );

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
        bool changeTorqueEnable( uint8_t value );
        bool analyzeServoResponse( std::string fun_name, bool *responses );


        /*
         * variables
         */
        /*
         * ROS Specific Stuff
         */
        ros::NodeHandle node_handle;
        /* Publishers */
        ros::Publisher servo_info_publisher;
        ros::Publisher servo_joint_publisher;
        /* Timers */
        ros::Timer servo_info_timer;

        /*
         * Dynamixel Stuff
         */
        DynamixelWorkbench _bench;
        std::vector<dynamixel_workbench_msgs::XH> servo_info;
        sensor_msgs::JointState joints;
        bool _valid;

};


#endif //ARM_MOTION_DYNAMIXELCONTROLLER_H
