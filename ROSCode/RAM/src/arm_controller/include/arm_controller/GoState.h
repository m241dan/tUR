//
// Created by korisd on 4/28/18.
//

#ifndef ARM_CONTROLLER_GOSTATE_H
#define ARM_CONTROLLER_GOSTATE_H

#include "ArmState.h"

#define ARRIVAL_THRESHOLD 10
#define MAX_MAG 24.0
#define MIN_MAG 10.0

typedef enum
{
    LOAD_WAYPOINT, ARM_TRAVELING, ARM_ARRIVED
} iGoState;


class GoState : public ArmState
{
    public:
        GoState( InputsTable *i );
        virtual std::string transition();
        virtual void action();
        virtual void onExit( std::string next_state );

    private:
        iGoState internalTransition();
        void internalAction();
        void forceTransition( iGoState transition_to );
        /* this above internal state machine only transitions from load, to travelling, to arrived */
        iGoState internal_state;
        geometry_msgs::Pose *present_waypoint; /* use this pointer to be for the front of the list, this helps make robust transitions  */
};


#endif //ARM_CONTROLLER_GOSTATE_H
