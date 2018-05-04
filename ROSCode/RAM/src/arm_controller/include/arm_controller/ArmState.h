//
// Created by korisd on 5/1/18.
//

#ifndef ARM_CONTROLLER_ARMSTATE_H
#define ARM_CONTROLLER_ARMSTATE_H

#include <vector>

#include "state_machine/State.h"
#include "shared_types.h"
#include "kinematics.h"

class ArmState : public State
{
    public:
        ArmState( InputsTable *i, std::string name );
        virtual std::string transition();
        std::vector<ServoCommand> getOutputs();
    protected:
        void torqueOn();
        void torqueOff();
        void holdPosition();
        void resetCommands();
        uint32_t radianToValue( double radian,  int32_t max_position, int32_t min_position, float max_radian = 3.14, float min_radian = -3.14 );
        ServoCommand generatePositionCommand( int id, double position );
        ServoCommand generateVelocityCommand( int id, int velocity );
        kinematics::Coordinates poseToCoordinates( geometry_msgs::Pose pose );

        InputsTable *inputs;
        std::vector<ServoCommand> outputs;
};


#endif //ARM_CONTROLLER_ARMSTATE_H
