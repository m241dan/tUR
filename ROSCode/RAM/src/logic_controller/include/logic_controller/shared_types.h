//
// Created by korisd on 5/10/18.
//

#ifndef SHARED_TYPES_H
#define SHARED_TYPES_H

#include <vector>
#include <string>
#include "logic_controller/action.h"
#include "logic_controller/trial.h"

typedef enum
{
    MANUAL_STATE, WAITING_STATE, LOADING_STATE, PERFORM_STATE, VERIFY_STATE, MAX_STATE
} LOGIC_STATE;

typedef struct inputs_table
{
    /* logic stuff */
    std::vector<Trial> trials_queue;
    Trial *present_trial;
    uint8_t trial_mode = 4;
    double valid_perform = 0;

    /* arm stuff */
    uint16_t arm_waypoint_queue_size;
    geometry_msgs::Pose present_kinematics;
    geometry_msgs::Pose goal_kinematics;
    uint8_t arm_desired_mode;
    std::string arm_state_machine_present_state;
} InputsTable;

/*
#define MANUAL_STATE "manual_state"
#define WAITING_STATE "waiting_state"
#define LOADING_STATE "loading_state"
#define PERFORM_STATE "perform_state"
#define VERIFY_STATE "verify_state"
*/


#endif //SHARED_TYPES_H
