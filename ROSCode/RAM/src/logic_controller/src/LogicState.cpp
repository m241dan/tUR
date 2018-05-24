//
// Created by korisd on 5/14/18.
//

#include "logic_controller/LogicState.h"

LogicState::LogicState( InputsTable *i, std::string name ) : State( name ), inputs(i)
{

}

geometry_msgs::PoseArray LogicState::getOutputs()
{
    return outputs;
}