//
// Created by korisd on 4/28/18.
//

#include "arm_controller/WaitingState.h"

WaitingState::WaitingState(InputsTable *i) : ArmState( i, WAITING_STATE )
{

}

void WaitingState::onEnter( std::string prev_state )
{
    outputs.clear();

    for( int i = ROTATION_SERVO; i < MAX_SERVO; i++ )
    {
        int id = i + 1;
        ServoCommand com;
        com.id = id;
        com.value = inputs->servos[i].Present_Position;
        com.command = "Goal_Position";
        outputs.push_back( com );
    }
}

std::string WaitingState::transition()
{
    std::string transition_to;

    switch( inputs->desired_mode )
    {
        default:
            transition_to = mode_state_strings[inputs->desired_mode];
            break;
        case GO_MODE:
        case GO_SYNCHRONIZED_MODE:
            if( inputs->waypoint_queue.size() > 0 )
                transition_to = mode_state_strings[inputs->desired_mode];
            break;
    }



    return transition_to;
}


