//
// Created by korisd on 6/5/18.
//

#ifndef ARM_MOTION_PATH_H
#define ARM_MOTION_PATH_H

#include "arm_motion/arm_motion_node.h"
#include <geometry_msgs/PoseArray.h>
#include <arm_motion/MotionMsg.h>
#include <sensor_msgs/JointState.h>
#include <tuple>

const double length1 = 2.6;
const double length2 = 15.465;
const double length3 = 8.927;
const double length4 = 11.055;

class Path
{
    public:
        Path( arm_motion::MotionMsg motion, geometry_msgs::Pose pose );
        std::vector<sensor_msgs::JointState> &getPath();
    protected:
        void generateFinalPose();
        void generatePathConstants();
        void generatePath();
        void pathToJointPositions();            //performs the IK
        void setJointPositionVelocities();
        void setJointPositionEffort();

        sensor_msgs::JointState inverseKinematics( geometry_msgs::Pose &pose );

        arm_motion::MotionMsg _motion_guidelines;
        geometry_msgs::Pose _present_pose;
        geometry_msgs::Pose _final_pose;
        double _A, _B, _C, _D;
        geometry_msgs::PoseArray _motion_path;
        std::vector<sensor_msgs::JointState> _output_joint_states;
};


#endif //ARM_MOTION_PATH_H
