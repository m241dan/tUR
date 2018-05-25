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
                ROS_INFO( "%s: Successful startup of Dynamixel Bus %s", __FUNCTION__, bus );
                if( verifyServos )
                {
                    ROS_INFO( "%s: Servos verified and online.", __FUNCTION__ );
                }
            }
            else
            {
                ROS_INFO( "%s: Failed to startup Dynamixel Bus %s", __FUNCTION__, bus );
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

        bool verifyServos()
        {
            bool success = true;

            if( _valid )
            {

                uint8_t num_servos;
                uint8_t servo_ids[4] = {0, 0, 0, 0};

                _bench.scan( servo_ids, &num_servos, 4 );

                if( num_servos != 4 )
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
            for( int i = ROTATION_SERVO; i < MAX_SERVO; i++ )
            {
                _bench.itemWrite()
            }
        }

        bool benchWrite( )
        {

        }

        bool validCommand( ServoCommand com )
        {
            bool valid = true;

            if( com.id < ROTATION_SERVO || com.id >= MAX_SERVO )
            {
                valid = false;
            }
            else if( com.command == "" )
            {
                valid = false;
            }
            else
            {

            }

            return valid;
        }
        /*
         * variables
         */
        DynamixelWorkbench _bench;
        bool _valid;

};


#endif //ARM_MOTION_DYNAMIXELCONTROLLER_H
