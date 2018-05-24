//
// Created by korisd on 5/11/18.
//

#include "logic_controller/trial.h"

Trial::Trial( std::string trial_name ) : name( trial_name ), action_tracker( 0 ), trial_complete( false )
{
    //load trial with Lua
    //load actions with Lua
    if( name == "discrete_w" )
    {
        Action action;
        action.eeo = 0.0;
        action.x = 18.0;
        action.y = 12.0;
        action.z = 10.0;
        action.velocity = 20;
        action.type = DISCRETE_W;
        action.precision = 5;
        action.shape = "linear";

        action_queue.push_back( action );

        action.y = -12.0;

        action_queue.push_back( action );

        action.z = 20.0;

        action_queue.push_back( action );

        action.y = 12.0;

        action_queue.push_back( action );
    }
    else if( name == "discrete_r" )
    {
        Action action;
        action.eeo = 0;
        action.x = 5.0;
        action.y = 5.0;
        action.z = 5.0;
        action.velocity = 20;
        action.type = DISCRETE_R;
        action.precision = 5;
        action.shape = "linear";

        action_queue.push_back( action );

        action.x = 0.0;
        action.z = 0.0;
        action.y = -10.0;

        action_queue.push_back( action );

        action.x = -5.0;
        action.y = 10.0;
        action.z = 5.0;
        action.shape = "linear";

        action_queue.push_back( action );
    }
    else
    {
        /* mixed */
        Action action;
        action.eeo = 0.0;
        action.x = 15.0;
        action.y = 0.0;
        action.z = 10.0;
        action.velocity = 20;
        action.type = DISCRETE_W;
        action.precision = 5;
        action.shape = "linear";

        action_queue.push_back( action );

        action.x = -5.0;
        action.y = -10.0;
        action.z = 0.0;
        action.type = DISCRETE_R;
        action.shape = "parabolic";

        action_queue.push_back( action );
    }
}

Trial::Trial( std::string trial_name, lua_State *lua, bool &success ) : name(trial_name), action_tracker(0), trial_complete( false )
{
    std::stringstream ss;
    ss << "/home/ubuntu/tUR/ROSCode/RAM/src/logic_controller/scripts/" << trial_name << ".lua";

    std::cout << "Path: " << ss.str().c_str() << std::endl;
    if( luaL_loadfile( lua, ss.str().c_str() ) || lua_pcall( lua, 0, 1, 0 ) )
    {
        messaging::errorMsg( __FUNCTION__, lua_tostring( lua, -1 ) );
        success = false;
    }
    else
    {
        /* factor to function */
        lua_len( lua, -1 );
        int size = lua_tointeger( lua, -1 );
        lua_pop( lua, 1 );


        for( int i = 1; i < size+1; i++ )
        {
            Action action;
            /* stack: trial table */
            /* get the action at i */
            lua_pushinteger( lua, i );
            lua_gettable( lua, -2 );

            /*
             * Velocity
             */
            /* stack: trial table -> action[i] */
            lua_pushstring( lua, "velocity" );
            /* stack: trial table -> action[i] -> "velocity" */
            lua_gettable( lua, -2 );
            /* stack: trial table -> action[i] -> velocity_num */
            action.velocity = (uint8_t )lua_tointeger( lua, -1 );
            lua_pop( lua, 1 );
            /* stack: trial table -> action[i] */

            /*
             * Type
             */
            lua_pushstring( lua, "type" );
            lua_gettable( lua, -2 );
            action.type = (ActionType)lua_tointeger( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * Precision
             */
            lua_pushstring( lua, "precision" );
            lua_gettable( lua, -2 );
            action.precision = (uint8_t)lua_tointeger( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * Shape
             */
            lua_pushstring( lua, "shape" );
            lua_gettable( lua, -2 );
            action.shape = std::string( lua_tostring( lua, -1 ) );
            lua_pop( lua, 1 );

            /*
             * EEO
             */
            lua_pushstring( lua, "eeo" );
            lua_gettable( lua, -2 );
            action.eeo = lua_tonumber( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * X
             */
            lua_pushstring( lua, "x" );
            lua_gettable( lua, -2 );
            action.x = lua_tonumber( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * Y
             */
            lua_pushstring( lua, "y" );
            lua_gettable( lua, -2 );
            action.y = lua_tonumber( lua, -1 );
            lua_pop( lua, 1 );

            /*
             * Z
             */
            lua_pushstring( lua, "z" );
            lua_gettable( lua, -2 );
            action.z = lua_tonumber( lua, -1 );
            lua_pop( lua, 2 ); /* pop the number and the action off */
            /* stack: trial table */
            action_queue.push_back( action );
        }
        lua_pop( lua, 1 );
        /* stack: nil */
        success = true;
    }
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

    if( action_queue.size() > 0 )
    {
        present_action = action_queue.at( action_tracker );
        switch( present_action.type )
        {
            default:
                break;
            case DISCRETE_R:
            {
                /*
                 * THIS DESCRIPTION MAY NOW BE OUT OF DATE
                 * ALSO NEEDS REFACTORING
                 *
                 * This essentially is generating points along a space curve that is either linear (t) or parabolic (t^2)
                 * r(t) = < At, Bt, Ct> or r(t) = < At^2, Bt^2, Ct^2 >
                 * To get the constants, we use the final position for the waypoint for the solution to the equation
                 * and set t equal to the max number of steps required to get there. So, for example,
                 * just if we want to move along a position 5 cm in the x direction. The other two components will be a
                 * zero change so we can ignore them. Thus, we can isolate the x for this example. Now, let's say we have
                 * our precision set to 2. What this signifies is that it will make the move 5 cm away in 2 steps.
                 * Now, we need to solve for A to complete our equation with the given information:
                 * deltaX = At or 5 = A2, thus, A = 2. Now, we generate our waypoints by plugging A back into the function
                 * and solving it as t increases from 1 to 2. r(1) = <2.5,0,0> and r(2) = <5,0,0>. Finally, for the actual
                 * waypoints, take current position and add each vector to generate the waypoints. The same is also done
                 * with the end effector, so technically there is a "4th" dimension.
                 */
                std::tuple<double, double, double, double> constants = generateConstants( present_action.x,
                                                                                          present_action.y,
                                                                                          present_action.z,
                                                                                          present_action.eeo );

                const double A = std::get<0>( constants );
                const double B = std::get<1>( constants );
                const double C = std::get<2>( constants );
                const double D = std::get<3>( constants );

                for( int i = 1; i <= present_action.precision; i++ )
                {
                    double t;
                    if( present_action.shape == "linear" )
                        t = (double) i;
                    else
                        t = (double) (i * i);

                    geometry_msgs::Pose pose = *present_kinematics;
                    pose.position.x += A * t;
                    pose.position.y += B * t;
                    pose.position.z += C * t;
                    pose.orientation.w += D * t;
                    pose.orientation.z = present_action.velocity;

                    waypoints.poses.push_back( pose );
                }
                break;
            }
            case DISCRETE_W:
            {
                std::tuple<double, double, double, double> constants = generateConstants(
                        (present_action.x - present_kinematics->position.x),
                        (present_action.y - present_kinematics->position.y),
                        (present_action.z - present_kinematics->position.z),
                        (present_action.eeo - present_kinematics->orientation.w));

                const double A = std::get<0>( constants );
                const double B = std::get<1>( constants );
                const double C = std::get<2>( constants );
                const double D = std::get<3>( constants );

                for( int i = 1; i <= present_action.precision; i++ )
                {
                    double t;
                    if( present_action.shape == "linear" )
                    {
                        t = (double) i;
                    }
                    else
                    {
                        t = (double) (i * i);
                    }

                    geometry_msgs::Pose pose = *present_kinematics;
                    pose.position.x += A * t;
                    pose.position.y += B * t;
                    pose.position.z += C * t;
                    pose.orientation.w += D * t;
                    pose.orientation.z = present_action.velocity;


                    waypoints.poses.push_back( pose );
                }
                break;
            }
        }

    }

    return waypoints;
}

bool Trial::isTrialComplete()
{
    return trial_complete;
}

bool Trial::isActionComplete()
{
    bool complete = false;

    complete = verifyAction();

    if( complete )
        nextAction();

    return complete;
}

void Trial::nextAction()
{
    if((this->action_tracker + 1) ==
       action_queue.size()) //remember we have zero indexing on vectors, vector of size 1 will have one thing at index 0, and tracker is the index
    {
        trial_complete = true;
    }
    else
    {
        this->action_tracker++;
        present_action = action_queue.at( action_tracker );
    }
}

std::tuple<double, double, double, double>
Trial::generateConstants( double new_x, double new_y, double new_z, double new_eeo )
{
    double A = 0.;
    double B = 0.;
    double C = 0.;
    double D = 0.;


    double t = 0.00001;
    if( present_action.shape == "linear" )
        t = (double) present_action.precision;
    else
        t = (double) (present_action.precision * present_action.precision);

    A = new_x / t;
    B = new_y / t;
    C = new_z / t;
    D = new_eeo / t;

    return std::make_tuple( A, B, C, D );
};

bool Trial::verifyAction()
{
    bool verified = false;

    switch( present_action.type )
    {
        default:
            break;
        case DISCRETE_W:
            if( fabs( present_kinematics->position.x - present_action.x ) < WORLD_ERROR &&
                fabs( present_kinematics->position.y - present_action.y ) < WORLD_ERROR &&
                fabs( present_kinematics->position.z - present_action.z ) < WORLD_ERROR &&
                fabs( present_kinematics->orientation.w - present_action.eeo ) < WORLD_ERROR )
            {
                verified = true;
            }

            break;
        case DISCRETE_R:
            verified = true;
            break;
        case VISION:
            break;
    }

    return verified;
}
