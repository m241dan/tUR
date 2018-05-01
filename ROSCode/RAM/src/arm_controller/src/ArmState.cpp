//
// Created by korisd on 5/1/18.
//

#include "arm_controller/ArmState.h"

ArmState:: ArmState( InputsTable *i, std::string name ) : State( name ), inputs(i) {}

std::string ArmState::transition()
{
    std::string transition_to = this->getIdentifier();

    switch( inputs->desired_mode )
    {
        default:
            break;
        case OFF_MODE:
            transition_to = OFF_STATE;
            break;
        case PAUSE_MODE:
            transition_to = PAUSE_STATE;
            break;
        case GO_MODE:
            transition_to = GO_STATE;
            break;
        case GO_SYNCHRONIZED_MODE:
            transition_to = GO_SYNCHRONIZED_STATE;
            break;

    }

    return transition_to;
}

std::vector<ServoCommand> ArmState::getOutputs() { return outputs; }