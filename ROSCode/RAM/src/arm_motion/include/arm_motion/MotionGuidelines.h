//
// Created by korisd on 5/11/18.
//

#ifndef ACTION_H
#define ACTION_H

#include <string>

typedef enum
{
    VISION, DISCRETE_R, DISCRETE_W, MAX_ACTION
} MotionType;

typedef struct motion_guidelines
{
    std::string name = "default-action";
    MotionType type = VISION;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double eeo = 0.0;
    double gripper_rot = 0.0;
    double gripper_lin = 0.0;
    uint8_t velocity = 20;
    uint8_t precision = 0;
    uint16_t smoothness = 150;
    uint16_t tolerance = 2;
    std::string shape = "linear";
    uint8_t tag_id = 0;
} MotionGuidelines;

#endif //ACTION_H
