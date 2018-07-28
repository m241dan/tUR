//
// Created by korisd on 5/25/18.
//

#include "arm_motion/MotionActor.h"

MotionActor::MotionActor( std::string name, DynamixelController &controller ) :
        action_server( node_handle, name, false ),
        servo_server( node_handle, name + "_servo", false ),
        action_name(name),
        _controller(controller)
{
    action_server.registerGoalCallback( boost::bind( &MotionActor::goalCallBack, this ) );
    action_server.registerPreemptCallback( boost::bind( &MotionActor::preemptCallBack, this ) );
    action_server.start();

    servo_server.registerGoalCallback( boost::bind( &MotionActor::servoGoalCallback, this ) );
    servo_server.registerPreemptCallback( boost::bind( &MotionActor::servoPreemptCallback, this ) );
    servo_server.start();

    _start_motion = node_handle.serviceClient<std_srvs::Empty>( "kinematics/start_motion" );
    _stop_motion = node_handle.serviceClient<std_srvs::Empty>("kinematics/stop_motion" );
    _kinematics_subscriber = node_handle.subscribe( "/kinematics/servo_based_fk", 10, &MotionActor::kinematicsTick, this );
}

void MotionActor::kinematicsTick( const geometry_msgs::Pose::ConstPtr &pose )
{
    if( action_server.isActive() )
    {
        motionMonitor();
    }

    if( servo_server.isActive() )
    {
        servoMonitor();
    }
}

void MotionActor::goalCallBack()
{
    /*setup the new motion*/
    joint_goals = action_server.acceptNewGoal()->joints;
    goal_step = -1;
    goal_max = (int16_t)joint_goals.size();

    std_srvs::Empty empty;
    _start_motion.call(empty);
}

void MotionActor::preemptCallBack()
{
    result.success = 0;
    action_server.setPreempted( result );
    _controller.holdPosition();
}

void MotionActor::motionMonitor()
{
    if( action_server.isActive() )
    {
        if( goal_step == -1 )
        {
            performMotionStep();
        }
        else
        {
          //  if( checkMotionStep()) // check for arrival
            if( !_controller.armMoving() )
            {
                goal_step++;
                if( goal_step < goal_max )
                {
                    /* if we haven't completed motion */
                    if( !performMotionStep())
                    {
                        result.success = 0;
                        action_server.setAborted( result );
                        _controller.holdPosition();
                    }
                    else
                    {
                       // feedback.on_step = goal_step;
                        action_server.publishFeedback( feedback );
                    }
                }
                else
                {
                    /* motion is completed */
                    result.success = 1;
                    action_server.setSucceeded( result );
                    std_srvs::Empty empty;
                    _stop_motion.call( empty );
                }
            }
        }
    }
}

bool MotionActor::checkMotionStep()
{
    bool arrived = true;
    std::vector<int32_t> servo_positions = _controller.getServoPositions();
    std::vector<int32_t> servo_goals = _controller.getServoGoals();

    for( int i = 0; i < MAX_ARM_SERVO; i++ )
    {
        auto tolerance = (int32_t)joint_goals[goal_step].effort[i]; //oddly this works out that we can use the same i for all three
        if( abs( servo_positions[i] - servo_goals[i] ) > tolerance )
        {
            ROS_INFO( "Servo[%d] not arrived.", i+1 );
            arrived = false;
            break;
        }
    }
    return arrived;
}

bool MotionActor::performMotionStep()
{
    bool success = true;
    if( goal_step != goal_max )
    {
        uint8_t id = 0;
        for( int x = 0; x < MAX_SERVO; x++ )
            _controller.loadDefault( ++id );
        id = 0;
        for( auto velocity : joint_goals[goal_step].velocity )
        {
            id++;
            if( !_controller.changeVelocity( id, (uint32_t )velocity ) )
            {
                success = false;
            }
        }

        id = 0;
        for( auto position : joint_goals[goal_step].position )
        {
            id++;
            if(! _controller.changePosition( id, position ) )
            {
                success = false;
            }
        }
    }
    else
    {
        success = false;
    }
    return success;
}

void MotionActor::servoGoalCallback()
{
    /*setup the new motion*/
    _goal = *servo_server.acceptNewGoal();
    goal_step = 0;
    goal_max = 1;

    std_srvs::Empty empty;
    _start_motion.call(empty);
}

void MotionActor::servoPreemptCallback()
{
    arm_motion::ServoMotionResult res;
    result.success = 0;
    servo_server.setPreempted( res );
}

void MotionActor::servoMonitor()
{
    if( servo_server.isActive() )
    {
        if( goal_step == -1 )
        {
            performServoStep();
        }
        else
        {
           // if( checkServoStep()) // check for arrival
            if( !_controller.armMoving() )
            {
                ROS_INFO( "Servo Motion Complete" );
                arm_motion::ServoMotionResult res;
                res.success = 1;
                servo_server.setSucceeded( res );
                std_srvs::Empty empty;
                _stop_motion.call( empty );
            }
        }
    }
}

bool MotionActor::checkServoStep()
{
    bool arrived = true;
    std::vector<int32_t> servo_positions = _controller.getServoPositions();
    std::vector<int32_t> servo_goals = _controller.getServoGoals();

    for( int i = 0; i < MAX_SERVO; i++ )
    {
        ROS_INFO( "Servo[%d] Position[%d] Goal[%d]", i + 1, servo_positions[i], servo_goals[i] );
        if( abs( servo_positions[i] - servo_goals[i] ) > 2 )
        {
            arrived = false;
            break;
        }
    }
    return arrived;
}

bool MotionActor::performServoStep()
{
    bool success = true;
    uint32_t velocity = _goal.velocity;
    if( _goal.type == SERVO_R )
    {
        std::vector<int32_t> servo_positions = _controller.getServoPositions();
        //velocity
        if( _goal.servo_one != 0 )
        {
            if( !_controller.changeVelocity( 1, velocity ) )
            {
                success = false;
            }
        }
        if( _goal.servo_two != 0 )
        {
            if( !_controller.changeVelocity( 2, velocity ) )
            {
                success = false;
            }
        }
        if( _goal.servo_three != 0 )
        {
            if( !_controller.changeVelocity( 3, velocity ) )
            {
                success = false;
            }
        }
        if( _goal.servo_four != 0 )
        {
            if( !_controller.changeVelocity( 4, velocity ) )
            {
                success = false;
            }
        }
        if( _goal.servo_five != 0 )
        {
            if( !_controller.changeVelocity( 5, velocity ) )
            {
                success = false;
            }
        }
        if( _goal.servo_six != 0 )
        {
            if( !_controller.changeVelocity( 6, velocity ) )
            {
                success = false;
            }
        }

        //position
        if( _goal.servo_one != 0 )
        {
            int32_t new_position = servo_positions[0] + _goal.servo_one;
            if( !_controller.changePosition( 1, new_position ) )
            {
                success = false;
            }
        }
        if( _goal.servo_two != 0 )
        {
            int32_t new_position = servo_positions[1] + _goal.servo_two;
            if( !_controller.changePosition( 2, new_position ) )
            {
                success = false;
            }
        }
        if( _goal.servo_three != 0 )
        {
            int32_t new_position = servo_positions[2] + _goal.servo_three;
            if( !_controller.changePosition( 3, new_position ) )
            {
                success = false;
            }
        }
        if( _goal.servo_four != 0 )
        {
            int32_t new_position = servo_positions[3] + _goal.servo_four;
            if( !_controller.changePosition( 4, new_position ) )
            {
                success = false;
            }
        }
        if( _goal.servo_five != 0 )
        {
            int32_t new_position = servo_positions[4] + _goal.servo_five;
            if( !_controller.changePosition( 5, new_position ) )
            {
                success = false;
            }
        }
        if( _goal.servo_six != 0 )
        {
            int32_t new_position = servo_positions[5] + _goal.servo_six;
            if( !_controller.changePosition( 6, new_position ) )
            {
                success = false;
            }
        }
    }
    else
    {
        for( uint8_t x = 0; x < MAX_SERVO; x++ )
        {
            _controller.changeVelocity( x+1, _goal.velocity );
        }
        if( (int32_t)_goal.servo_one != 0 )
            _controller.changePosition( 1, (int32_t )_goal.servo_one );
        if( (int32_t)_goal.servo_two != 0 )
            _controller.changePosition( 2, (int32_t )_goal.servo_two );
        if( (int32_t)_goal.servo_three != 0 )
            _controller.changePosition( 3, (int32_t )_goal.servo_three );
        if( (int32_t)_goal.servo_four != 0 )
            _controller.changePosition( 4, (int32_t )_goal.servo_four );
        if( (int32_t)_goal.servo_five != 0 )
            _controller.changePosition( 5, (int32_t )_goal.servo_five );
        if( (int32_t)_goal.servo_six != 0 )
            _controller.changePosition( 6, (int32_t )_goal.servo_six );

    }

    return success;
}
