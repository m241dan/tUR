//
// Created by korisd on 5/14/18.
//

#ifndef LOGIC_CONTROLLER_PERFORMSTATE_H
#define LOGIC_CONTROLLER_PERFORMSTATE_H

#include "logic_controller/LogicState.h"

class PerformState : public LogicState
{
    public:
        PerformState( InputsTable *i );
        virtual std::string transition();
    private:
};


#endif //LOGIC_CONTROLLER_PERFORMSTATE_H
