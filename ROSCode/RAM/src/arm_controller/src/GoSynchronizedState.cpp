//
// Created by korisd on 4/28/18.
//

#include "arm_controller/GoSynchronizedState.h"

GoSynchronizedState::GoSynchronizedState(InputsTable *i) : GoState( i, GO_SYNCHRONIZED_STATE )
{

}

std::string GoSynchronizedState::transition()
{
    std::string transition_to = getIdentifier();

    if( inputs->waypoint_queue.size() == 0 )
    {
        transition_to = WAITING_STATE;
    }
    else
    {
        switch( inputs->desired_mode )
        {
            case OFF_MODE:
                transition_to = OFF_STATE;
                break;
            case PAUSE_MODE:
                transition_to = PAUSE_STATE;
                break;
            case GO_MODE:
                transition_to = GO_STATE;
                break;
        }
    }
    return transition_to;
}
void GoSynchronizedState::forceTransition(iGoState transition_to)
{
    iGoState prev_state = internal_state;

    internal_state = transition_to;

    /* handle the onEnter / onExit bits for an internal state machine */
    if( internal_state != prev_state )
    {
        /* internal onExit */
        switch( prev_state )
        {
            default: break;
        }

        /* internal onEnter */
        switch( internal_state )
        {
            default: break;
            case ARM_TRAVELING:
                resetCommands();
                torqueOn();

                if( present_waypoint )
                {
                    /*
                     * velocity is in 0.229 rpm
                     * servos are capable of 0 ~ 4095 positions
                     * 0.088 degees per "position"
                     */
                    double max_velocity = inputs->waypoint_queue.front().orientation.z;

                    if( max_velocity == 0 )
                        max_velocity = MAX_VELOCITY;

                    /* convert pose to kinematics::Coordinates */
                    kinematics::Coordinates goal_coordinates = poseToCoordinates( inputs->waypoint_queue.front() );
                    /* grab the EE orientation */
                    double goal_EE_orientation= inputs->waypoint_queue.front().orientation.w;

                    /* do the inverse kinematics */
                    kinematics::Joints goal_joint_positions = kinematics::inverseKinematics( goal_coordinates, goal_EE_orientation );

                    /*
                     * have to flip joint 4 (wrist) due to our setup
                     */
                    goal_joint_positions._4 *= -1;
                    uint16_t gjp_in_dynamixel[MAX_SERVO] = { 0, 0, 0, 0 }; /* goal_joint_positions in dynamixel units */

                    /* do position message first */
                    for( int i = ROTATION_SERVO; i < MAX_SERVO; i++ )
                    {
                        int servo_id = i + 1;
                        ServoCommand com = generatePositionCommand( servo_id, goal_joint_positions[i] );
                        gjp_in_dynamixel[i] = com.value;
                        std::cout << "com value " << com.value << std::endl;
                        outputs.push_back( com );
                    }

                    uint16_t servo_distance_to_goal[MAX_SERVO] = { 0, 0, 0, 0 };
                    uint16_t max_distance = 0;

                    /* calculate the distance to travel for each servo */
                    for( int i = ROTATION_SERVO; i < MAX_SERVO; i++ )
                    {
                        /* servo present position */
                        uint16_t spp = inputs->servos[i].Present_Position;
                        uint16_t distance = abs( spp - gjp_in_dynamixel[i] );
                        if( distance > max_distance )
                            max_distance = distance;
                        servo_distance_to_goal[i] = abs( spp - gjp_in_dynamixel[i] );
                    }

                    /* figure out our maximum travel time at "max" velocity */
                    double max_time = max_distance / max_velocity;

                    /* now figure out our velocity for each servo proportional to their distance */
                    uint16_t servo_velocity[MAX_SERVO] = { 1, 1, 1, 1 };
                    for( int i = ROTATION_SERVO; i < MAX_SERVO; i++ )
                    {
                        servo_velocity[i] = (uint16_t )( (double)servo_distance_to_goal[i] / max_time );
                        if( servo_velocity[i] <= 0 )
                            servo_velocity[i] = 1;
                    }

                    /* do velocity message first */
                    for( int i = ROTATION_SERVO; i < MAX_SERVO; i++ )
                    {
                        int servo_id = i + 1;
                        ServoCommand com = generateVelocityCommand( servo_id, servo_velocity[i]);
                        std::cout << "sending velocity " << servo_velocity[i] << std::endl;
                        outputs.push_back( com );
                    }
                }
                break;
        }
    }
}