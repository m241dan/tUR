//
// Created by korisd on 5/28/18.
//

#ifndef ARM_MOTION_ARMKINEMATICS_H
#define ARM_MOTION_ARMKINEMATICS_H

#include "arm_motion/arm_motion_node.h"
#include <math.h>
#include <eigen3/Eigen/Eigen>

/* should read this in from Lua */
#define length1 2.6
#define length2 12.6
#define length3 9.01
#define length4 11.00
typedef Eigen::Matrix<float,4,4> Matrix4;

class ArmKinematics
{
    public:
        ArmKinematics( std::string test );
    protected:
        /*
         * Functions
         */
        void setupSubscribers();
        void setupPublishers();
        void servoInfoHandler( const sensor_msgs::JointState::ConstPtr &joints );

        /* Forward Kinematics */
        void updateServoForwardKinematics();
        void publishServoForwardKinematics();

        Matrix4 HomogenousDHMatrix( double theta, double alpha, double r, double d );

        /*
         * Variables
         */
        /* ROS */
        ros::NodeHandle _node_handle;
        ros::Publisher _servo_fk_publisher;
        ros::Subscriber _joints_sub;
        sensor_msgs::JointState _joints;

        /* Kinematics */
        geometry_msgs::Pose _servo_based_fk;

};


#endif //ARM_MOTION_ARMKINEMATICS_H
