//
// Created by korisd on 5/14/18.
//

#include "logic_controller/WaitingState.h"

WaitingState::WaitingState( InputsTable *i ) : LogicState( i, WAITING_STATE )
{

}

std::string WaitingState::transition()
{
    std::string transition_to = getIdentifier();

    if( inputs->trials_queue.size() > 0 )
    {
        transition_to = LOADING_STATE;
    }

    return transition_to;
}
