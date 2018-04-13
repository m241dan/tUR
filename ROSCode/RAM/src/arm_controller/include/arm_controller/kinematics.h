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

#define length1 5.0
#define length2 10.0
#define length3 10.0
#define length4 10.0

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


    Joints inverseKinematics( Coordinates desired_coords, double EE_orientation_desired )
    {
        double x_q = desired_coords.x - length4*cos(EE_orientation_desired);
        double z_q = desired_coords.z - length1 - length4*sin(EE_orientation_desired);
        double CQ = sqrt( x_q * x_q + z_q * z_q );
        double CP = sqrt( desired_coords.x * desired_coords.x + desired_coords.z * desired_coords.z );

        double gamma = atan2( desired_coords.z - length1, desired_coords.x );
        double beta = acos( ( CQ * CQ + CP * CP - length4 * length4 ) / ( 2 * CQ * CP ) );
        if( isnan( beta ) )
            beta = 0;
        double alpha = acos( ( CQ * CQ + length2 * length2 - length3 * length3 ) / ( 2 * CQ * length2) );

        double theta_1 = atan2( desired_coords.y, desired_coords.x );
        double theta_2 = alpha + beta + gamma;
        double theta_3 = (-1) * ( M_PI -  acos( ( length2 * length2 + length3 * length3 - CQ * CQ ) / ( 2 * length2 * length3 ) ) );
        double theta_4 = EE_orientation_desired - theta_2 - theta_3;

        Joints set_one;
        set_one._1 = theta_1;
        set_one._2 = theta_2;
        set_one._3 = theta_3;
        set_one._4 = theta_4;

        return set_one;
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
