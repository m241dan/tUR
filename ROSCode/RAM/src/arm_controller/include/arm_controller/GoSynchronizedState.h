//
// Created by korisd on 4/28/18.
//

#ifndef ARM_CONTROLLER_GOSYNCHRONIZEDSTATE_H
#define ARM_CONTROLLER_GOSYNCHRONIZEDSTATE_H

#include "GoState.h"

class GoSynchronizedState : public GoState
{
    public:
        GoSynchronizedState( InputsTable *i );
        virtual std::string transition();
    protected:
        void forceTransition( iGoState transition_to );

};


#endif //ARM_CONTROLLER_GOSYNCHRONIZEDSTATE_H
