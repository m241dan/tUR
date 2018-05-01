//
// Created by korisd on 4/28/18.
//

#include "arm_controller/PauseState.h"

PauseState::PauseState(InputsTable *i) : ArmState(i, PAUSE_STATE)
{

}

void PauseState::onEnter(std::string prev_state)
{
    outputs.clear();

    for( int i = ROTATION_SERVO; i < MAX_SERVO; i++ )
    {
        int id = i + 1;
        ServoCommand com;
        com.id = id;
        com.value = inputs->servos[i].Present_Position;
        com.command = "Goal_Position";
        outputs.push_back( com );
    }
}