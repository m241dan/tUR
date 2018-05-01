//
// Created by korisd on 4/28/18.
//

#ifndef ARM_CONTROLLER_WAITINGSTATE_H
#define ARM_CONTROLLER_WAITINGSTATE_H

#include "ArmState.h"

class WaitingState : public ArmState
{
    public:
        WaitingState( InputsTable *i );
        virtual void onEnter( std::string prev_state );
        virtual std::string transition();
};


#endif //ARM_CONTROLLER_WAITINGSTATE_H
