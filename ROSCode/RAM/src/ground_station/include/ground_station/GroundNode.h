//
// Created by korisd on 7/7/18.
//

#ifndef GROUNDNODE_H
#define GROUNDNODE_H

#include <ground_node.h>
#include <fstream>
#include <chrono>
#include <ground_station/Ambient.h>
#include <ground_station/BBox.h>
#include <ground_station/PathLog.h>
#include <ground_station/ArmStatus.h>
#include <ground_station/NetworkHealth.h>
#include <fstream>

class GroundNode
{
    public:
        GroundNode();

    protected:
        std::string _log;
        ros::NodeHandle _node_handle;
        ros::Timer      _gtp_timer;
        int _gtp_rate;
        void timerCallback( const ros::TimerEvent &event );

        ros::Subscriber _command_subscriber;
        ros::Subscriber _serial_output;
        ros::Publisher _serial_input;
        void commandCallback( const ground_station::HaspCommand::ConstPtr &msg );
        void outputCallback( const std_msgs::UInt8MultiArray::ConstPtr &msg );
        template<typename packet_type>
        packet_type extractPacket( data_packet &data, uint16_t &offset );
        std::vector<uint8_t> _buffer;

        ros::Publisher _ambient;
        ros::Publisher _bbox;
        ros::Publisher _arm_status;
        ros::Publisher _arm_path;
        ros::Publisher _network_status;

        void publishAmbient( ambient_packet &packet );
        void publishBBox( bbox_packet &packet );
        void publishArmStatus( arm_packet &packet );
        void publishPathLog( pathlog_packet &packet );
        void publishNetworkStatus( network_packet &packet );
};


#endif //HASP_GROUND_STATION_GROUNDNODE_H
