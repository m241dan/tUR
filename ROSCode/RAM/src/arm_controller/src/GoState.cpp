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
    std::cout << "doing go action!" << std::endl;
    if( inputs->waypoint_queue.size() > 0 )
    {

        /* if traveling, do nothing */
        if (fabs(inputs->current_position.position.x - inputs->waypoint_queue.front().position.x) < ARRIVAL_THRESHOLD &&
            fabs(inputs->current_position.position.y - inputs->waypoint_queue.front().position.y) < ARRIVAL_THRESHOLD &&
            fabs(inputs->current_position.position.z - inputs->waypoint_queue.front().position.z) < ARRIVAL_THRESHOLD &&
            fabs(inputs->current_position.orientation.x - inputs->waypoint_queue.front().orientation.x) < ARRIVAL_THRESHOLD)
        {
            inputs->waypoint_queue.erase(inputs->waypoint_queue.begin());
        }

        if (inputs->waypoint_queue.size() > 0)
        {
            resetCommands();
            torqueOn();

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

           desired_joints._4 *= -1;

            /* do position messages first */
            for (int i = ROTATION_SERVO; i < MAX_SERVO; i++)
            {
                ServoCommand com;
                int id = i + 1;

                com.id = id;
                com.command = "Goal_Position";
                com.value = radianToValue(desired_joints[i], 4095, 0 );
                std::cout << "GOJoint[" << i + 1 << "]:" << com.value << std::endl;
                com.value_in_radians = true;

                outputs.push_back(com);
            }

            /* now does velocity messages */
            for (int i = ROTATION_SERVO; i < MAX_SERVO; i++)
            {
                ServoCommand com;
                int id = i + 1;

                com.id = id;
                com.command = "Profile_Velocity";
                com.value = velocity;

                outputs.push_back(com);
            }

        }
    }
}
