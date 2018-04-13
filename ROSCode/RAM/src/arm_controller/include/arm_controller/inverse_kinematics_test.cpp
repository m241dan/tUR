#include <iostream>
#include "kinematics.h"

using namespace kinematics;
using namespace std;

int main()
{
    Joints joints;
    joints._4 = -M_PI_2;
    joints._3 = -M_PI_2;
    joints._2 = M_PI_2;
    std::tuple<Coordinates,double> ret = forwardKinematics( joints );
    Coordinates coords = get<0>( ret );
    double EE_orientation = get<1>( ret );

    cout << "From some Forward Kinematics: " << std::endl;
    cout << " - Theta1: " << joints._1 << endl;
    cout << " - Theta2: " << joints._2 << endl;
    cout << " - Theta3: " << joints._3 << endl;
    cout << " - Theta4: " << joints._4 << endl;
    cout << " - X:" << coords.x << endl;
    cout << " - Y:" << coords.y << endl;
    cout << " - Z:" << coords.z << endl;
    cout << " - E:" << EE_orientation << endl;


    tuple<Joints,Joints> IKs = inverseKinematics( coords, EE_orientation );
    Joints ik_joints_1 = get<0>( IKs );
    Joints ik_joints_2 = get<1>( IKs );

    cout << "IK One:" << endl;
    cout << " - Theta1: " << ik_joints_1._1 << endl;
    cout << " - Theta2: " << ik_joints_1._2 << endl;
    cout << " - Theta3: " << ik_joints_1._3 << endl;
    cout << " - Theta4: " << ik_joints_1._4 << endl;

    cout << "IK Two:" << endl;
    cout << " - Theta1: " << ik_joints_2._1 << endl;
    cout << " - Theta2: " << ik_joints_2._2 << endl;
    cout << " - Theta3: " << ik_joints_2._3 << endl;
    cout << " - Theta4: " << ik_joints_2._4 << endl;

    return 0;
}
