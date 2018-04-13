#include <iostream>
#include "kinematics.h"

using namespace kinematics;
using namespace std;

int main()
{
    Joints joints;
    joints._4 = 0;
    joints._3 = 0; //M_PI_2;
    joints._2 = M_PI_4;
    joints._1 = M_PI_4;
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

    Joints ik_joints_1 = inverseKinematics( coords, EE_orientation );

    cout << "IK One:" << endl;
    cout << " - Theta1: " << ik_joints_1._1 << endl;
    cout << " - Theta2: " << ik_joints_1._2 << endl;
    cout << " - Theta3: " << ik_joints_1._3 << endl;
    cout << " - Theta4: " << ik_joints_1._4 << endl;

    ret = forwardKinematics( ik_joints_1 );
    coords = get<0>( ret );
    EE_orientation = get<1>( ret );

    cout << "Re-Forward: " << std::endl;
    cout << " - X:" << coords.x << endl;
    cout << " - Y:" << coords.y << endl;
    cout << " - Z:" << coords.z << endl;
    cout << " - E:" << EE_orientation << endl;

    return 0;
}
