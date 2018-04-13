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

    cout << "X:" << coords.x << endl;
    cout << "Y:" << coords.y << endl;
    cout << "Z:" << coords.z << endl;
    cout << "E:" << EE_orientation << endl;

    return 0;
}
