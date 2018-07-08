//
// Created by korisd on 7/7/18.
//

#ifndef GROUNDNODE_H
#define GROUNDNODE_H

#include <ground_node.h>

class GroundNode
{
    public:
        GroundNode( std::string port, int baud );

    protected:
        serial::Serial _serial_handle;

        ros::NodeHandle _node_handle;
        ros::Timer      _gtp_timer;
        int _gtp_rate;
        void timerCallback( const ros::TimerEvent &event );

        ros::Subscriber _command_subscriber;
        void commandCallback( const ram_network::HaspCommand::ConstPtr &msg );
};


#endif //HASP_GROUND_STATION_GROUNDNODE_H
