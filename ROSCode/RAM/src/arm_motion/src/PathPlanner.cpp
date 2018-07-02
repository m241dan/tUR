//
// Created by korisd on 6/8/18.
//

#include "arm_motion/PathPlanner.h"

PathPlanner::PathPlanner( std::string name )
{
    setupSubscribers();
    setupServices();
}

void PathPlanner::setupSubscribers()
{
    _servo_based_fk = _node_handle.subscribe( "kinematics/servo_based_fk", 10, &PathPlanner::servoBasedFK, this );
}

void PathPlanner::setupServices()
{
    _service = _node_handle.advertiseService( "plan_a_path", &PathPlanner::planPath, this );
}

void PathPlanner::servoBasedFK( const geometry_msgs::Pose::ConstPtr &pose )
{
    _servo_based_fk_pose = *pose;
}

bool PathPlanner::planPath( arm_motion::PathService::Request &guidelines, arm_motion::PathService::Response &path )
{
    Path new_path( guidelines.motion, guidelines.present_position );
    path.joints = new_path.getPath();
    return true;
}