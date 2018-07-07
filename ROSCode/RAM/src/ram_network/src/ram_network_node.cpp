#include <ram_network/ram_network.h>
#include <ram_network/NetworkNode.h>

int main( int argc, char *argv[] )
{
    ros::init( argc, argv, "ram_network" );
    ROS_INFO( "Ram Network Node: Boot Starting" );
    NetworkNode network;
    ROS_INFO( "Ram Network Node: Boot Complete" );
    ROS_INFO( "Ram Network Node: Spinning with no issues" );
    ros::spin();
    return 0;
}
