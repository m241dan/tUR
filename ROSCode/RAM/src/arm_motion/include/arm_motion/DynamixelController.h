//
// Created by korisd on 5/25/18.
//

#ifndef ARM_MOTION_DYNAMIXELCONTROLLER_H
#define ARM_MOTION_DYNAMIXELCONTROLLER_H

#include "arm_motion/arm_motion_node.h"
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

class DynamixelController
{
    public:
        DynamixelController( std::string bus )
        {
            _valid = _bench.begin( bus.c_str() );

            if( _valid )
            {
                ROS_INFO( "%s: Successful startup of Dynamixel Bus %s", __FUNCTION__, bus.c_str() );
                if( verifyServos( MAX_SERVO ) )
                {
                    ROS_INFO( "%s: Servos verified and online.", __FUNCTION__ );
                }
            }
            else
            {
                ROS_INFO( "%s: Failed to startup Dynamixel Bus %s", __FUNCTION__, bus.c_str() );
            }
        }

    protected:
        /*
         * functions
         */
        void validateBus()
        {
            ROS_INFO( "Bus has been validated." );
            _valid = true;
        }

        void invalidateBus()
        {
            ROS_INFO( "Dynamixel bus has been invalidated." );
            _valid = false;
        }

        bool verifyServos( int expected_number )
        {
            bool success = true;

            if( _valid )
            {

                uint8_t num_servos;
                uint8_t servo_ids[4] = {0, 0, 0, 0};

                _bench.scan( servo_ids, &num_servos, expected_number );

                if( num_servos != expected_number )
                {
                    ROS_INFO( "%s: invalid number of servos found on scan.", __FUNCTION__ );
                    invalidateBus();
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
                            ROS_INFO( "%s: found a servo with the ID of zero.", __FUNCTION__ );
                            success = false;
                        }
                    }
                }
            }
            else
            {
                ROS_INFO( "%s: bus is invalid.", __FUNCTION__ );
                success = false;
            }
            return success;
        }

        void initializeServos()
        {
            ServoCommand max_vel;
            ServoCommand pid_i_gain;

            max_vel.command = "Velocity_Limit";
            max_vel.value = MAX_VELOCITY;
            pid_i_gain.command = "Position_I_Gain";
            pid_i_gain.value = PID_I_GAIN;

            for( int i = ROTATION_SERVO; i < MAX_SERVO; i++ )
            {
                int id = i + 1;
                max_vel.id = id;
                pid_i_gain.id = id;

                benchWrite( max_vel );
                benchWrite( pid_i_gain );
            }
        }

        bool benchWrite( ServoCommand com )
        {
            bool success = false;

            if( validCommand( com ) )
                success = _bench.itemWrite( com.id, com.command.c_str(), com.value );

            if( !success )
                ROS_INFO( "%s: failed to write command { %d, %s, %d }", __FUNCTION__, com.id, com.command.c_str(), com.value );

            return success;
        }

        bool validCommand( ServoCommand com )
        {
            bool valid = false;

            /*
             * Assume the command is bad.
             * If it is in the valid ID range and command is not ""
             * Then, check the command field to make sure its valid
             * If the command shows up in the valid_commands table, it's good.
             */
            if( com.id < ROTATION_SERVO || com.id > MAX_SERVO )
            {
                ROS_INFO( "%s: ID[%d] is outside acceptable ranges.", __FUNCTION__, com.id );
            }
            else if( com.command == "" )
            {
                ROS_INFO( "%s: Attempting to send a \"\" command ", __FUNCTION__ );
            }
            else
            {
                for( int i = 0; i < MAX_COMMAND; i++ )
                {
                    if( com.command == valid_commands[i] )
                    {
                        valid = true;
                        break;
                    }
                }
                if( !valid )
                    ROS_INFO( "%s: Command \"%s\" is invalid for X Series.", __FUNCTION__, com.command.c_str() );
            }

            return valid;
        }
        int32_t benchRead( uint8_t id, std::string command )
        {
            return _bench.itemRead( id, command.c_str() );
        }

        /*
         * variables
         */
        DynamixelWorkbench _bench;
        bool _valid;

};


#endif //ARM_MOTION_DYNAMIXELCONTROLLER_H
