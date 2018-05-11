//
// Created by korisd on 5/11/18.
//

#include "logic_controller/trial.h"

Trial::Trial( std::string trial_name ) : name( trial_name )
{
    //load trial with Lua
    //load actions with Lua

    Action test_action;
    test_action.name = "test-action";
    test_action.x = 5.0;
    test_action.y = 5.0;
    test_action.z = 5.0;
    test_action.precision = 10;
    test_action.shape = "linear";

    action_queue.push_back( test_action );
    present_action = &action_queue.front();
}

void Trial::setPresentKinematics( geometry_msgs::Pose *pk )
{
    present_kinematics = pk;
}

void Trial::setPresentDetections( std::vector<geometry_msgs::PoseStamped> *pd )
{
    present_detections = pd;
}

geometry_msgs::PoseArray Trial::generateWaypoints()
{
    geometry_msgs::PoseArray waypoints;

    if( action_queue.size() > 0 && *present_action == action_queue.front())
    {
        switch( present_action->type )
        {
            default:
                break;
            case DISCRETE_R:
            {
                double t = 0.00001;
                if( present_action->shape == "linear" )
                    t = (double) present_action->precision;
                else
                    t = (double) (present_action->precision * present_action->precision);

                const double A = present_action->x / t;
                const double B = present_action->y / t;
                const double C = present_action->z / t;
                const double D = present_action->eeo / t;

                for( int i = 1; i <= t; i++ )
                {
                    double k;
                    if( present_action->shape == "linear" )
                        k = (double)i;
                    else
                        k = (double)( i * i );

                    geometry_msgs::Pose pose = *present_kinematics;
                    pose.position.x += A * k;
                    pose.position.y += B * k;
                    pose.position.z += C * k;
                    pose.orientation.w += D * k;

                    waypoints.poses.push_back( pose );

                }
            }
        }

    }

    return waypoints;
}

bool Trial::isTrialComplete()
{

}

bool Trial::isActionComplete()
{

}

