//
// Created by korisd on 5/14/18.
//

#ifndef LOGIC_CONTROLLER_LOADINGSTATE_H
#define LOGIC_CONTROLLER_LOADINGSTATE_H

#include "logic_controller/LogicState.h"

class LoadingState : public LogicState
{
    public:
        LoadingState( InputsTable *i );
        virtual void action();
        virtual std::string transition();
    protected:
};


#endif //LOGIC_CONTROLLER_LOADINGSTATE_H
