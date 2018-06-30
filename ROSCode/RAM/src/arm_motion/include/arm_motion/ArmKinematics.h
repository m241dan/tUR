//
// Created by korisd on 5/28/18.
//

#ifndef ARM_MOTION_ARMKINEMATICS_H
#define ARM_MOTION_ARMKINEMATICS_H

#include "arm_motion/arm_motion_node.h"
#include <math.h>
#include <eigen3/Eigen/Eigen>

/* should read this in from Lua */
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
        void camOneTagHandler( const apriltags_ros::AprilTagDetectionArrayConstPtr &tags );

        /* Forward Kinematics */
        void updateServoForwardKinematics();
        void publishServoForwardKinematics();

        /* Vision Forward Kinematics */
        void updateVisionForwardKinematics();
        void publishVisionForwardKinematics();

        void convertTagsQuaternionToRPY();

        Matrix4 HomogenousDHMatrix( double theta, double alpha, double r, double d );
        /*
         * Variables
         */
        /* ROS */
        ros::NodeHandle _node_handle;
        ros::Publisher _servo_fk_publisher;
        ros::Publisher _vision_fk_publisher;
        ros::Subscriber _joints_sub;
        ros::Subscriber _camera_one_tags;
        sensor_msgs::JointState _joints;

        /* Kinematics */
        geometry_msgs::Pose _servo_based_fk;
        apriltags_ros::AprilTagDetectionArray _tags_seen;

};


#endif //ARM_MOTION_ARMKINEMATICS_H
