//
// Created by korisd on 5/28/18.
//

#ifndef ARM_MOTION_ARMKINEMATICS_H
#define ARM_MOTION_ARMKINEMATICS_H

#include "arm_motion/arm_motion_node.h"
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <std_srvs/Empty.h>
#include <arm_motion/StartTrial.h>

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

        void updateServos( const arm_motion::ArmInfoConstPtr &msg );

        /* Forward Kinematics */
        void updateServoForwardKinematics();
        void publishServoForwardKinematics();

        /* Vision Forward Kinematics */
        void updateVisionForwardKinematics();
        void publishVisionForwardKinematics();

        void convertTagsQuaternionToRPY();

        Matrix4 HomogenousDHMatrix( double theta, double alpha, double r, double d );

        bool startMotion( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res );
        bool stopMotion( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res );
        bool startTrial( arm_motion::StartTrial::Request &req, arm_motion::StartTrial::Response &res );
        bool stopTrial( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res );

        void faultCallback( const std_msgs::UInt8ConstPtr &msg );
        /*
         * Variables
         */
        /* ROS */
        ros::NodeHandle _node_handle;
        ros::Publisher _servo_fk_publisher;
        ros::Publisher _vision_fk_publisher;
        ros::Subscriber _joints_sub;
        ros::Subscriber _camera_one_tags;
        ros::Subscriber _servo_info_sub;
        sensor_msgs::JointState _joints;
        ros::ServiceServer _start_motion;
        ros::ServiceServer _stop_motion;
        ros::ServiceServer _start_trial;
        ros::ServiceServer _stop_trial;
        ros::Publisher _motion_data_publisher;
        ros::Publisher _trial_data_publisher;
        ros::Subscriber _fault_subscriber;

        /* Kinematics */
        geometry_msgs::Pose _servo_based_fk;
        apriltags_ros::AprilTagDetectionArray _tags_seen;

        std::vector<dynamixel_workbench_msgs::XH> _servos;

        arm_motion::TrialData _present_trial;
        arm_motion::MotionData _present_motion;

        void clockCallback( const rosgraph_msgs::ClockConstPtr &msg );
        ros::Subscriber _clock_sub;
        int _clock = 0;
        bool _fault;



};


#endif //ARM_MOTION_ARMKINEMATICS_H
