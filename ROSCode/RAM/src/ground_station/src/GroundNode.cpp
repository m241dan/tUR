//
// Created by korisd on 7/7/18.
//

#include "GroundNode.h"

GroundNode::GroundNode() : _node_handle("~")
{
    _node_handle.param( "refreshrate", _gtp_rate, 60 );
    _gtp_timer = _node_handle.createTimer( ros::Duration(_gtp_rate),boost::bind( &GroundNode::timerCallback, this, _1 ) );

    _serial_output = _node_handle.subscribe( "/serial_output", 10, &GroundNode::outputCallback, this );
    _command_subscriber = _node_handle.subscribe( "/command", 10, &GroundNode::commandCallback, this );
    _serial_input = _node_handle.advertise<std_msgs::ByteMultiArray>( "/serial_input", 10 );
}

void GroundNode::timerCallback( const ros::TimerEvent &event )
{
    std_msgs::ByteMultiArray output;
    gtp spoof;

    snprintf( spoof.data, sizeof( spoof.data ), "%f,$GPGGA,202212.00,3024.7205,N,09110.7264,W,1,06,1.69,00061,M,-025,M,,*51,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,", ros::Time::now().toSec() );

    uint8_t *ptr = (uint8_t *)&spoof;
    for( int x = 0; x < sizeof( gtp ); x++ )
        output.data.push_back( *ptr++ );

    _serial_input.publish(output);
}

void GroundNode::commandCallback( const ram_network::HaspCommand::ConstPtr &msg )
{
    std_msgs::ByteMultiArray output;
    ground_command com;
    com.command[0] = msg->com_id;
    com.command[1] = msg->com_param;

    auto *ptr = (uint8_t *)&com;
    for( int x = 0; x < sizeof( ground_command ); x++ )
        output.data.push_back( *ptr++ );

    _serial_input.publish(output);

}

void GroundNode::outputCallback( const std_msgs::ByteMultiArray::ConstPtr &msg )
{
    for( auto val : msg->data )
    {
        _buffer[_buffer_index++] = (uint8_t) val;
    }

    if( _buffer_index >= 8 )
    {
        struct test {
            uint32_t one;
            uint8_t two;
        };

        test my_test;
        auto *ptr = (uint8_t *)&my_test;

        for( int x = 0; x < sizeof( test ); x++ )
        {
            *ptr++ = _buffer[x];
        }
        if( my_test.one == 328 && my_test.two == 60 )
        {
            ROS_ERROR( "!!!TEST PASSED!!!" );
            _buffer_index = 0;
            memset( _buffer, 0, sizeof( MAX_BUF ) );
        }
    }

}