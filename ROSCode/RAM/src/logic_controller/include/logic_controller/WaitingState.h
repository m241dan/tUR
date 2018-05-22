//
// Created by korisd on 5/14/18.
//

#ifndef LOGIC_CONTROLLER_WAITINGSTATE_H
#define LOGIC_CONTROLLER_WAITINGSTATE_H

#include "logic_controller/LogicState.h"

/*
 * This state is mostly a do nothing state.
 * If the arm has no trials queued up and not in manual mode
 * If there are trials queued up, this state will transition to load actions state.
 * If it is told to be in manual mode, it should transition to that.
 */
class WaitingState : public LogicState
{
    public:
        WaitingState( InputsTable *i );
        virtual std::string transition();
    private:
};


#endif //LOGIC_CONTROLLER_WAITINGSTATE_H
