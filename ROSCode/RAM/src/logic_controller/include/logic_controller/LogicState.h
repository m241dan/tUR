//
// Created by korisd on 5/10/18.
//

#ifndef LOGICSTATE_H
#define LOGICSTATE_H

#include "state_machine/State.h"
#include "logic_controller/shared_types.h"
#include <string>

class LogicState : public State
{
    public:
        LogicState( InputsTable *i, std::string name );
        virtual geometry_msgs::PoseArray getOutputs();

    protected:
        InputsTable *inputs;
        geometry_msgs::PoseArray outputs;
};

#endif //LOGICSTATE_H
