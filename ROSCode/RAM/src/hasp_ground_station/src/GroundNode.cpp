//
// Created by korisd on 7/7/18.
//

#include "GroundNode.h"

GroundNode::GroundNode( std::string port, int baud ) : _node_handle("~"), _serial_handle(port, baud)
{
    try
    {
        _serial_handle.open();
    }
    catch ( std::exception &e )
    {
        ROS_INFO( "Unhandled Exception: %s", e.what() );
        ros::shutdown();
        exit(0);
    }
    _node_handle.param( "refresh rate", _gtp_rate, 60 );
    _gtp_timer = _node_handle.createTimer( ros::Duration(_gtp_rate),boost::bind( &GroundNode::timerCallback, this, _1 ) );
    _command_subscriber = _node_handle.subscribe( "/command", 10, &GroundNode::commandCallback, this );
}

void GroundNode::timerCallback( const ros::TimerEvent &event )
{
    gtp spoof;
    snprintf( spoof.data, sizeof( spoof.data ), "%f,$GPGGA,202212.00,3024.7205,N,09110.7264,W,1,06,1.69,00061,M,-025,M,,*51,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,", ros::Time::now().toSec() );

    _serial_handle.write( (uint8_t *)&spoof, sizeof( gtp ) );
}

void GroundNode::commandCallback( const ram_network::HaspCommand::ConstPtr &msg )
{
    ground_command com;
    com.command[0] = msg->com_id;
    com.command[1] = msg->com_param;

    _serial_handle.write( (uint8_t *)&com, sizeof( ground_command ) );
}