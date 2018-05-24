//
// Created by korisd on 4/28/18.
//

#ifndef ARM_CONTROLLER_GOSTATE_H
#define ARM_CONTROLLER_GOSTATE_H

#include "ArmState.h"

#define TRAVELING_ARRIVAL_THRESHOLD 150
#define ARRIVAL_THRESHOLD 2

typedef enum
{
    LOAD_WAYPOINT, ARM_TRAVELING, ARM_ARRIVED
} iGoState;


class GoState : public ArmState
{
    public:
        GoState( InputsTable *i );
        GoState( InputsTable *i, std::string state_name );
        virtual std::string transition();
        virtual void action();
        virtual void onExit( std::string next_state );

    protected:
        iGoState internalTransition();
        void internalAction();
        virtual void forceTransition( iGoState transition_to );
        /* this above internal state machine only transitions from load, to travelling, to arrived */
        iGoState internal_state;
        geometry_msgs::Pose *present_waypoint; /* use this pointer to be for the front of the list, this helps make robust transitions  */
};


#endif //ARM_CONTROLLER_GOSTATE_H
