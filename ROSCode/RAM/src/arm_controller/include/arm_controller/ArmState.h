//
// Created by korisd on 5/1/18.
//

#ifndef ARM_CONTROLLER_ARMSTATE_H
#define ARM_CONTROLLER_ARMSTATE_H

#include "state_machine/State.h"
#include "shared_types.h"
#include <vector>

class ArmState : public State
{
    public:
        ArmState( InputsTable *i, std::string name );
        virtual std::string transition();
        std::vector<ServoCommand> getOutputs();
    protected:
        InputsTable *inputs;
        std::vector<ServoCommand> outputs;
};


#endif //ARM_CONTROLLER_ARMSTATE_H
