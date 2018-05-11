//
// Created by korisd on 5/11/18.
//

#ifndef TRIAL_H
#define TRIAL_H

#include <geometry_msgs/Pose.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

class Trial
{
    public:
        Trial( std::string trial_name );
        void setPresentKinematics( geometry_msgs::Pose *pk );
        void setPresentDetections( std::vector<geometry_msgs::PoseStamped> *pd );
        std::vector<geometry_msgs::PoseArray> generateWaypoints();
        bool isComplete();
    private:
        geometry_msgs::Pose *present_kinematics;
        std::vector<geometry_msgs::PoseStamped> *present_detections;
        //std::vector<Action> action_queue;
};

#endif //TRIAL_H
