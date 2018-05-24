//
// Created by korisd on 5/14/18.
//

#include "logic_controller/PerformState.h"

PerformState::PerformState( InputsTable *i ) : LogicState( i, PERFORM_STATE )
{

}

std::string PerformState::transition()
{
    std::string transition_to = getIdentifier();

    if( inputs->arm_waypoint_queue_size == 0 )
    {
        transition_to = VERIFY_STATE;
    }

    return transition_to;
}