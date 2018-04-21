//
// Created by korisd on 4/20/18.
//

#ifndef RAM_ARM_CONTROLLER_H
#define RAM_ARM_CONTROLLER_H

#include <ros/ros.h>
#include <string>
#include "dynamixel_sdk/port_handler.h"

/*
 * Publishers
 */
ros::Publisher queue_size;
ros::Publisher state_machine_mode;
ros::Publisher desired_mode;
ros::Publisher current_kinematics;
ros::Publisher goal_kinematics;
ros::Publisher servo_one_info;
ros::Publisher servo_two_info;
ros::Publisher servo_three_info;
ros::Publisher servo_four_info;

/*
 * Subscribers
 */

ros::Subscriber enqueue_waypoint;
ros::Subscriber reset_queue;
ros::Subscriber operation_mode;

void setupPublishers( ros::NodeHandle &ros_handle  );
void setupSubscribers( ros::NodeHandle &ros_handle );

#endif //RAM_ARM_CONTROLLER_H
