//
// Created by korisd on 4/28/18.
//

#ifndef ARM_CONTROLLER_OFFSTATE_H
#define ARM_CONTROLLER_OFFSTATE_H

#include "ArmState.h"

class OffState : public ArmState
{
    public:
        OffState( InputsTable *i );
};


#endif //ARM_CONTROLLER_OFFSTATE_H
