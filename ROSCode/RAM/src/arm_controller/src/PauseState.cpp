//
// Created by korisd on 4/28/18.
//

#include "arm_controller/PauseState.h"

PauseState::PauseState(InputsTable *i) : ArmState(i, PAUSE_STATE)
{

}

void PauseState::onEnter(std::string prev_state)
{
    resetCommands();
    torqueOn();
    holdPosition();
}