//
// Created by korisd on 4/28/18.
//

#include "arm_controller/OffState.h"

OffState::OffState(InputsTable *i) : ArmState(i, OFF_STATE )
{
    for( int i = 1; i < MAX_SERVO; i++ )
    {
        ServoCommand com;
        com.id = i;
        com.command = "Torque_Enable";
        com.value = 0;

        outputs.push_back( com );
    }
}