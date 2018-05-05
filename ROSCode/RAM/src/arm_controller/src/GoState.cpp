//
// Created by korisd on 4/28/18.
//

#include "arm_controller/GoState.h"

GoState::GoState( InputsTable *i ) : ArmState( i, GO_STATE )
{

}

std::string GoState::transition()
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
            case GO_SYNCHRONIZED_MODE:
                transition_to = GO_SYNCHRONIZED_STATE;
                break;
        }
    }
    return transition_to;
}

void GoState::action()
{
    forceTransition(internalTransition());
    internalAction();
}

void GoState::onExit( std::string next_state )
{
    present_waypoint = nullptr;
    forceTransition( LOAD_WAYPOINT );
}

iGoState GoState::internalTransition()
{
    iGoState transition_to = internal_state;

    switch( internal_state )
    {
        default: break;
        case LOAD_WAYPOINT:
            /*
             * if the arm is in a load_waypoint state,
             * stay there until present_waypoint is not null.
             * If it is not null, should be moving to the arm_traveling state
             */
            if( present_waypoint )
            {
                transition_to = ARM_TRAVELING;
            }
            break;
        case ARM_TRAVELING:
        {
            /*
             * check for arrival,
             * if arrived, transition to arm_arrived,
             * otherwise stay in ARM_TRAVELING state
             */
            if( inputs->waypoint_queue.size() == 0 )
            {
                /* check to see if the queue has been cleared */
                transition_to = ARM_ARRIVED;
            }
            else
            {
                bool has_arrived = true;
                for (int i = ROTATION_SERVO; i < MAX_SERVO; i++)
                {
                    double to_goal = abs( inputs->servos[i].Present_Position - inputs->servos[i].Goal_Position );
                    std::cout << "ToGoal[" << i << "]: " << to_goal << std::endl;
                    if( to_goal > ARRIVAL_THRESHOLD)
                    {
                        has_arrived = false;
                        break;
                    }

                }
                if (has_arrived)
                {
                    transition_to = ARM_ARRIVED;
                }
            }
            break;
        }
        case ARM_ARRIVED:
            /*
             * once the present_waypoint is null, should transition to load_waypoint
             */
            if( !present_waypoint )
            {
                transition_to = LOAD_WAYPOINT;
            }
            break;
    }
    return transition_to;
}

void GoState::internalAction()
{
    switch( internal_state )
    {
        default: break;
        case LOAD_WAYPOINT:
            /* loads a waypoint into our current waypoint which generates our kinematics */
            if( !present_waypoint && inputs->waypoint_queue.size() > 0 )
            {
                double pose_magnitude = poseMagnitude( inputs->waypoint_queue.front() );
                if( pose_magnitude > MAX_MAG || pose_magnitude < MIN_MAG )
                {
                    messaging::errorMsg( __FUNCTION__, "Goal Pose violates min/max conditions" );
                    inputs->waypoint_queue.erase( inputs->waypoint_queue.begin() );
                }
                else
                {
                    present_waypoint = &inputs->waypoint_queue.front();
                }
            }
            break;
        case ARM_TRAVELING:
            /* should essentially be a no action state */

            break;
        case ARM_ARRIVED:
            /* should set preset_waypoint to null, should consider adding a checker to make sure this only runs once */
            present_waypoint = nullptr;
            if( inputs->waypoint_queue.size() > 0 )
            {
                /* have to check this, incase the transition to arrived was from a clear */
                inputs->waypoint_queue.erase( inputs->waypoint_queue.begin() );
            }
            break;

    }

}

void GoState::forceTransition( iGoState transition_to )
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
                    double velocity = inputs->waypoint_queue.front().orientation.z;
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

                    /* do position message first */
                    for( int i = ROTATION_SERVO; i < MAX_SERVO; i++ )
                    {
                        int servo_id = i + 1;
                        ServoCommand com = generatePositionCommand( servo_id, goal_joint_positions[i] );
                        outputs.push_back( com );
                    }

                    /* do velocity message first */
                    for( int i = ROTATION_SERVO; i < MAX_SERVO; i++ )
                    {
                        int servo_id = i + 1;
                        ServoCommand com = generateVelocityCommand( servo_id, velocity );
                        outputs.push_back( com );
                    }
                }
                break;
        }
    }

}