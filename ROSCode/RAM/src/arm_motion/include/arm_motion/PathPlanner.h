//
// Created by korisd on 6/8/18.
//

#ifndef ARM_MOTION_PATHPLANNER_H
#define ARM_MOTION_PATHPLANNER_H

#include "arm_motion/arm_motion_node.h"
#include "arm_motion/MotionMsg.h"
#include "arm_motion/PathService.h"
#include "arm_motion/Path.h"
#include <geometry_msgs/PoseArray.h>

class PathPlanner
{
    public:
        PathPlanner( std::string name );
    protected:
        void setupSubscribers();
        void setupServices();
        void servoBasedFK( const geometry_msgs::Pose::ConstPtr &pose );
        bool planPath( arm_motion::PathService::Request &guidelines, arm_motion::PathService::Response &path );

        geometry_msgs::Pose _servo_based_fk_pose;
        /*
         * ROS Stuff
         */
        ros::NodeHandle _node_handle;
        ros::Subscriber _servo_based_fk;
        ros::ServiceServer _service;

};

#endif //ARM_MOTION_PATHPLANNER_H
