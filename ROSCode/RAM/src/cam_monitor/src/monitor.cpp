#include <cam_monitor/CamMonitor.h>

int main( int argc, char *argv[] )
{
    ros::init( argc, argv, "cam_monitor" );
    CamMonitor();
    ros::spin();
    return 0;
}
