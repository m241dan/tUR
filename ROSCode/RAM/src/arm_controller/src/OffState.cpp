//
// Created by korisd on 4/28/18.
//

#include "arm_controller/OffState.h"

OffState::OffState(InputsTable *i) : ArmState(i, OFF_STATE )
{
    resetCommands();
    torqueOff();
}