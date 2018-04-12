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
    struct Coordinates
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Joints
    {
        double _1 = 0.0;
        double _2 = 0.0;
        double _3 = 0.0;
        double _4 = 0.0;
    };


    std::tuple< Joints, Joints > inverseKinematics( Coordinates desired_coords, double EE_orientation_desired )
    {
        double theta_1 = atan2( desired_coords.y / desired_coords.x );
        double c = coords.z - length1 - length4*sin(EE_orientation_desired);
        double A = coords.x - length4*cos(theta_1)*cos(EE_orientation_desired);
        double B = coords.y - length4*sin(theta_1)*cos(EE_orientation_desired);
        double C = coords.z - length1 - length4*sin(EE_orientation_desired);
        double theta_3 = acos( (A^2 + B^2 + C^2 - length2^2 - length3^2 ) / ( 2 * length2 * length3 ) );
        double a = length3*sin(theta_3);
        double b = length2 + length3*cos(theta_3);
        double r = sqrt( a^2 + b^2 );
        double theta_2_low = atan2( )


    };

    std::tuple< Coordinates, double > forwardKinematics( Joints joints )
    {
        Coordinates coords;
        double calc = length2*cos( joints._2 ) + length3*cos( joints._2 + joints._3 );
        double EE_orientation= joints._2 + joints._3 + joints._4;

        coords.x = cos( joints._1 ) * calc + length4*cos( joints._1 )*cos( EE_orientation );
        coords.y = sin( joints._1 ) * calc + length4*sin( joints._1 )*sin( EE_orientation );
        coords.z = ( length1 + length2*sin( joints._2 ) + length3*sin( joints._2 + joints._3 ) )
                   + length4*sin( EE_orientation );

        return std::make_tuple( coords, EE_orientation );
    };
}

#endif //RAM_KINEMATICS_H
