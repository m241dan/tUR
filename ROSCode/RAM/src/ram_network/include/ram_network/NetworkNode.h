//
// Created by korisd on 7/5/18.
//

#ifndef NETWORKNODE_H
#define NETWORKNODE_H

#include <ram_network/ram_network.h>
#include <queue>

class NetworkNode
{
    public:
        NetworkNode();
    protected:
        void setupServices(); // for commanding
        void setupSerialConnection();
        void setupI2CConnections();
        void setupPublishers();
        void setupTimers();


        void openAdaI2C();
        void openBBoxI2C();


        void networkLoop( const ros::TimerEvent &event );
        void handleSerial();
        void handleAda();
        void handleBBox();

        void downlinkPacket();
        void buildPacket();

        ArduinoRegisters                _registers;
        fdHandles                       _handles;

        std::queue<ground_command>      _ada_commands;
        std::queue<ground_command>      _bbox_commands;
        std::queue<ground_command>      _cam_commands;
        std::queue<ground_command>      _arm_commands;
        /*
         * ROS Stuff
         */
        ros::NodeHandle                 _node_handle;
        ros::Timer                      _network_loop;
        ros::Publisher                  _clock_publisher;
        rosgraph_msgs::Clock            _clock;



};


#endif
