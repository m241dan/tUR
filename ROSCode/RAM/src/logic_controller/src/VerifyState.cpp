//
// Created by korisd on 5/14/18.
//

#include "logic_controller/VerifyState.h"

VerifyState::VerifyState( InputsTable *i ) : LogicState( i, VERIFY_STATE ), verification_complete(false)
{

}

std::string VerifyState::transition()
{
    std::string transition_to = getIdentifier();

    Trial &present_trial = inputs->trials_queue.front();

    if( present_trial.isActionComplete() )
    {
        transition_to
    }
    else
    {
        transition_to = LOADING_STATE;
    }

    return transition_to;
}
