
#include <GroundNode.h>
#include <string>

int main( int argc, char *argv[] )
{
    ros::init( argc, argv, "ground_station_node" );
    GroundNode node;
    ros::spin();
    return 0;
}