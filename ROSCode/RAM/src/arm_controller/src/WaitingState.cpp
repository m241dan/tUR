//
// Created by korisd on 4/28/18.
//

#include "arm_controller/WaitingState.h"

WaitingState::WaitingState(InputsTable *i) : ArmState( i, WAITING_STATE )
{

}

void WaitingState::onEnter( std::string prev_state )
{
    resetCommands();
    torqueOn();
    holdPosition();
}

std::string WaitingState::transition()
{
    std::string transition_to = getIdentifier();

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


