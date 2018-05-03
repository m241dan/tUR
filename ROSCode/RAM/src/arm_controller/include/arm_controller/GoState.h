//
// Created by korisd on 4/28/18.
//

#ifndef ARM_CONTROLLER_GOSTATE_H
#define ARM_CONTROLLER_GOSTATE_H

#include "ArmState.h"

#define ARRIVAL_THRESHOLD 0.5

class GoState : public ArmState
{
    public:
        GoState( InputsTable *i );
        virtual std::string transition();
        virtual void action();
    private:

};


#endif //ARM_CONTROLLER_GOSTATE_H
