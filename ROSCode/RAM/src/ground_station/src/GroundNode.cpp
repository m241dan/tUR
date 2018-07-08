//
// Created by korisd on 7/7/18.
//

#include "GroundNode.h"

GroundNode::GroundNode() : _node_handle("~")
{
    _node_handle.param( "refreshrate", _gtp_rate, 60 );
    _gtp_timer = _node_handle.createTimer( ros::Duration(_gtp_rate),boost::bind( &GroundNode::timerCallback, this, _1 ) );

    _command_subscriber = _node_handle.subscribe( "/command", 10, &GroundNode::commandCallback, this );
    _serial_output = _node_handle.advertise<std_msgs::ByteMultiArray>( "/serial_input", 10 );
}

void GroundNode::timerCallback( const ros::TimerEvent &event )
{
    std_msgs::ByteMultiArray output;
    gtp spoof;

    snprintf( spoof.data, sizeof( spoof.data ), "%f,$GPGGA,202212.00,3024.7205,N,09110.7264,W,1,06,1.69,00061,M,-025,M,,*51,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,", ros::Time::now().toSec() );

    uint8_t *ptr = (uint8_t *)&spoof;
    for( int x = 0; x < sizeof( gtp ); x++ )
        output.data.push_back( *ptr++ );

    _serial_output.publish(output);
}

void GroundNode::commandCallback( const ram_network::HaspCommand::ConstPtr &msg )
{
    ground_command com;
    com.command[0] = msg->com_id;
    com.command[1] = msg->com_param;

    //_serial_handle.write( (uint8_t *)&com, sizeof( ground_command ) );
    ROS_INFO( "Writing... Commmand: ID[%d] Param[%d]", (int)com.command[0], (int)com.command[1] );
}