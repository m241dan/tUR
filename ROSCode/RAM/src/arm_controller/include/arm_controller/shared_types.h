//
// Created by korisd on 4/28/18.
//

#ifndef ARM_CONTROLLER_SHARED_TYPES_H
#define ARM_CONTROLLER_SHARED_TYPES_H

typedef struct inputs_table
{
    std::vector<geometry_msgs::Pose> waypoint_queue;
    dynamixel_workbench_msgs::XH servos[MAX_SERVO];
    geometry_msgs::Pose current_position;

} InputsTable;

#endif //ARM_CONTROLLER_SHARED_TYPES_H
