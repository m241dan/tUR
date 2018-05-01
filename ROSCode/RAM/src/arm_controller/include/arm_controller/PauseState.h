//
// Created by korisd on 4/28/18.
//

#ifndef ARM_CONTROLLER_PAUSESTATE_H
#define ARM_CONTROLLER_PAUSESTATE_H

#include "ArmState.h"

class PauseState : public ArmState
{
    public:
        PauseState( InputsTable *i );
        virtual void onEnter( std::string prev_state );
};


#endif //ARM_CONTROLLER_PAUSESTATE_H
