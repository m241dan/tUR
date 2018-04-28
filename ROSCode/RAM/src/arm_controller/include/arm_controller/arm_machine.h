//
// Created by korisd on 4/28/18.
//

#ifndef ARM_CONTROLLER_ARM_MACHINE_H
#define ARM_CONTROLLER_ARM_MACHINE_H

#include "state_machine/StateMachine.h"
#include "shared_types.h"

class ArmMachine : public StateMachine
{
    public:
        ArmMachine( InputsTable *i ) : inputs(i) {}
    private:
        InputsTable *inputs;

};

#endif //ARM_CONTROLLER_ARM_MACHINE_H
