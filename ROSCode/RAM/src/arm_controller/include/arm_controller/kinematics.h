//
// Created by korisd on 4/11/18.
//

/*
 * Inverse Kinematics for our 4 DOF arm.
 * Math borrowed from https://www.researchgate.net/publication/279201859_Kinematics_Modeling_of_a_4-DOF_Robotic_Arm
 * Because it closely resembles ours
 */
#ifndef RAM_KINEMATICS_H
#define RAM_KINEMATICS_H

#include <tuple>
#include <math.h>

/*
 * Lengths in cm
 * length1 is distance from base servo to shoulder
 * length2 shoulder to elbow
 * length3 elbow to wrist
 * length4 wrist to end-effector
 */

#define length1 5
#define length2 10
#define length3 10
#define length4 10

namespace kinematics
{
    struct Coordinate
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct JointPositions
    {
        double joint_1 = 0.0;
        double joint_2 = 0.0;
        double joint_3 = 0.0;
        double joint_4 = 0.0;
    };

    /*
    std::tuple< JointPositions, JointPositions > inverseKinematics( Coordinate desired_coordinates )
    {

    };
*/
    std::tuple< Coordinate, double > forwardKinematics( JointPositions current_joints )
    {
        Coordinate coords;
        double EE_orientation = 0;

        double calc = length2*cos( current_joints.joint_2 ) + length3*cos( current_joints.joint_2 + current_joints.joint_3 );
        double angle_sum = current_joints.joint_2 + current_joints.joint_3 + current_joints.joint_4 );

        coords.x = cos( current_joints.joint_1 ) * calc + length4*cos( current_joints.joint_1)*cos( angle_sum );
        coords.y = sin( current_joints.joint_1 ) * calc + length4*sin( current_joints.joint_1)*sin( angle_sum );
        coords.z = ( length1 + length2*sin( current_joints.joint_2 ) + length3*sin( current_joints.joint_2 + current_joints.joint_3 ) )
                   + length4*sin( angle_sum );
        EE_orientation = angle_sum; //sort of a dumb step, will resolve later

        return std::make_tuple( coords, EE_orientation );
    };
}

#endif //RAM_KINEMATICS_H
