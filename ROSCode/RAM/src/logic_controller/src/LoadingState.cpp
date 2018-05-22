//
// Created by korisd on 5/14/18.
//

#include "logic_controller/LoadingState.h"

LoadingState::LoadingState( InputsTable *i ) : LogicState( i, LOADING_STATE )
{

}

void LoadingState::action()
{
    if( inputs->trials_queue.size() != 0 )
    {
        Trial &present_trial = inputs->trials_queue.front();
        outputs = present_trial.generateWaypoints();
    }
}

std::string LoadingState::transition()
{
    std::string transition_to = getIdentifier();

    if( inputs->trials_queue.size() == 0 )
    {
        transition_to = WAITING_STATE;
    }
    else
    {
        transition_to = PERFORM_STATE;
    }

    return transition_to;
}