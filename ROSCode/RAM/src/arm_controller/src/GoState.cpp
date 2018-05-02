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
    /* if traveling, do nothing */
    if( ( inputs->current_position.orientation.x - inputs->waypoint_queue.front().position.x ) < ARRIVAL_THRESHOLD &&
            ( inputs->current_position.orientation.y - inputs->waypoint_queue.front().position.y ) < ARRIVAL_THRESHOLD &&
            ( inputs->current_position.orientation.z - inputs->waypoint_queue.front().position.z ) < ARRIVAL_THRESHOLD &&
            ( inputs->current_position.orientation.x - inputs->waypoint_queue.front().orientation.x ) < ARRIVAL_THRESHOLD )

    {
        inputs->waypoint_queue.erase( inputs->waypoint_queue.begin() );

        if( inputs->waypoint_queue.size() > 0 )
        {
            outputs.clear();

            /* velocity is in 0.229 rpm */
            /* servos are capable of 0 ~ 4095 positions */
            /* 0.088 degees per "position" */

            double velocity = inputs->waypoint_queue.front().orientation.y;

            kinematics::Coordinates desired_coordinates;
            desired_coordinates.x = inputs->waypoint_queue.front().position.x;
            desired_coordinates.y = inputs->waypoint_queue.front().position.y;
            desired_coordinates.z = inputs->waypoint_queue.front().position.z;

            kinematics::Joints desired_joints = kinematics::inverseKinematics(desired_coordinates,
                                                                              inputs->waypoint_queue.front().orientation.x);

            /* do position messages first */
            for (int i = ROTATION_SERVO; i < MAX_SERVO; i++)
            {
                ServoCommand com;
                int id = i + 1;

                com.id = id;
                com.command = "Goal_Position";
                com.value = desired_joints[i];
                com.value_in_radians = true;

                outputs.push_back( com );
            }

            /* now does velocity messages */
            for( int i = ROTATION_SERVO; i < MAX_SERVO; i++ )
            {
                ServoCommand com;
                int id = i + 1;

                com.id = id;
                com.command = "Profile_Velocity";
                com.value = velocity;

                outputs.push_back( com );
            }

        }
    }
}
