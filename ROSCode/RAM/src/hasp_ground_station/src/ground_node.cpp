
#include <GroundNode.h>
#include <string>

int main( int argc, char *argv[] )
{
    std::string port;
    int baud;
    ros::init( argc, argv, "ground_station_node" );
    ros::NodeHandle nh("~");
    nh.param( "port", port, std::string( "/dev/ttyUSB0" ) );
    nh.param( "baud", baud, 4800 );
    GroundNode node( port, baud );
    ros::spin();
    ROS_INFO( "shutting down" );
    return 0;
}