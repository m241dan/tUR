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
#include "eigen3/Eigen/Eigen"

/*
 * Lengths in cm
 * length1 is distance from base servo to shoulder
 * length2 shoulder to elbow
 * length3 elbow to wrist
 * length4 wrist to end-effector
 */

#define length1 3.0
#define length2 9.7
#define length3 9.01
#define length4 10.00

using namespace Eigen;

typedef Matrix<float,4,4> Matrix4;

#define DegToRad(d)	(float)((float)d*(M_PI/180.0))
#define RadToDeg(r)	(float)((float)r*(180.0/M_PI))

Matrix4 HomogenousDHMatrix( float theta, float alpha, float r, float d );

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
        double operator[] (int x ) {
            switch( x )
            {
                case 0:
                    return _1;
                case 1:
                    return _2;
                case 2:
                    return _3;
                case 3:
                    return _4;

            }
        }
    };


    Joints inverseKinematics( Coordinates desired_coords, double EE_orientation_desired );
    std::tuple< Coordinates, double > forwardKinematics( Joints joints );
    Matrix4 HomogenousDHMatrix( float theta, float alpha, float r, float d );
}



#endif //RAM_KINEMATICS_H
