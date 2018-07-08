
#include <GroundNode.h>
#include <string>

int main( int argc, char *argv[] )
{
    std::string port;
    int baud;
    ros::init( argc, argv, "ground_node" );
    ros::NodeHandle nh("~");
    nh.param( "port", port, std::string( "/dev/ttyUSB0" ) );
    nh.param( "baud", baud, 4800 );
    GroundNode node( port, baud );
    ros::spin();
    return 0;
}