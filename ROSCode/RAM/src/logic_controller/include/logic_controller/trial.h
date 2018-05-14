//
// Created by korisd on 5/11/18.
//

#ifndef TRIAL_H
#define TRIAL_H

#include <geometry_msgs/Pose.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include "logic_controller/action.h"
#include <tuple>


class Trial
{
    public:
        Trial( std::string trial_name );
        void setPresentKinematics( geometry_msgs::Pose *pk );
        void setPresentDetections( std::vector<geometry_msgs::PoseStamped> *pd );
        geometry_msgs::PoseArray generateWaypoints();  //generates waypoints for an action based on present_kinematics and present_detections
        bool isTrialComplete(); //decides if there is more actions that this trial needs to do
        bool isActionComplete(); //verifies the action
    private:
        std::tuple<double,double,double,double> generateConstants();
        geometry_msgs::Pose *present_kinematics;
        std::vector<geometry_msgs::PoseStamped> *present_detections;
        std::vector<Action> action_queue;
        Action *present_action;
        std::string name;
};

#endif //TRIAL_H
