//
// Created by korisd on 4/28/18.
//

#ifndef ARM_CONTROLLER_OFFSTATE_H
#define ARM_CONTROLLER_OFFSTATE_H

#include "state_machine/State.h"
#include "shared_types.h"

class OffState : public State
{
    public:
        OffState( InputsTable *i ) : inputs(i)
        {
            for( int i = 1; i < MAX_SERVO; i++ )
            {
                ServoCommand com;
                com.id = i;
                com.command = "Torque_Enable"
                com.value = 0;
            }
        }
        virtual std::string transition();
        std::vector<ServoCommand> getOutputs() { return outputs; }
    private:
        InputsTable *inputs;
        std::vector<ServoCommand> outputs;

};


#endif //ARM_CONTROLLER_OFFSTATE_H
