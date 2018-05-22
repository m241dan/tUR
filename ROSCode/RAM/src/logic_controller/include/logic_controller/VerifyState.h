//
// Created by korisd on 5/14/18.
//

#ifndef LOGIC_CONTROLLER_VERIFYSTATE_H
#define LOGIC_CONTROLLER_VERIFYSTATE_H

#include "logic_controller/LogicState.h"

class VerifyState : public LogicState
{
    public:
        VerifyState( InputsTable *i );
        virtual std::string transition();
    private:
        bool verification_complete;
};


#endif //LOGIC_CONTROLLER_VERIFYSTATE_H
