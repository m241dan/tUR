//
// Created by korisd on 5/11/18.
//

#ifndef ACTION_H
#define ACTION_H

#include <string>

typedef enum
{
    VISION, DISCRETE_R, DISCRETE_W, MAX_ACTION
} ActionType;

typedef struct trial_action
{
    std::string name = "default-action";
    ActionType type = VISION;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double eeo = 0.0;
    double gripper_rot = 0.0;
    double gripper_lin = 0.0;
    uint8_t velocity = 20;
    uint8_t precision = 0;
    std::string shape = "linear";
    uint8_t tag_id = 0;
} Action;

#endif //ACTION_H
