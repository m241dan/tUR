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

extern "C" {
    #include "lua.h"
    #include "lauxlib.h"
    #include "lualib.h"
}

#define WORLD_ERROR 0.1

class Trial
{
    public:
        Trial( std::string trial_name );
        void setPresentKinematics( geometry_msgs::Pose *pk );
        void setPresentDetections( std::vector<geometry_msgs::PoseStamped> *pd );
        geometry_msgs::PoseArray generateWaypoints();  //generates waypoints for an action based on present_kinematics and present_detections
        bool isTrialComplete(); //decides if there is more actions that this trial needs to do
        bool isActionComplete(); //verifies the action
        void nextAction();

    private:
        std::tuple<double,double,double,double> generateConstants( double new_x, double new_y, double new_z, double new_eeo );
        bool verifyAction();

        geometry_msgs::Pose *present_kinematics;
        std::vector<geometry_msgs::PoseStamped> *present_detections;
        std::vector<Action> action_queue;
        Action present_action; /* points to the action_queue action */
        uint8_t action_tracker; /* this should determine where the present_action pointer points */
        std::string name;
        bool trial_complete;
};

#endif //TRIAL_H
