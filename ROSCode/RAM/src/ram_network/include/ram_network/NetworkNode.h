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
        void setupSubscribers(); // for simulating commanding
        void setupServices(); // for commanding
        void startSerialAndI2C();
        void setupI2CConnections();
        void setupPublishers();
        void setupTimers();

        int openSerialConnection();
        int openAdaI2C();
        int openBBoxI2C();


        void networkLoop( const ros::TimerEvent &event );
        void handleSerial();
        void parseSerial();
        void handleCommand( ground_command com );
        void handleGTP( gtp time );
        void handleAda();
        void handleBBox();
        void handleDownlink();

        void downlinkPacket();
        void buildPacket();

        NetworkHealth                   _health;
        ArduinoRegisters                _registers;
        fdHandles                       _handles;

        std::queue<ground_command>      _ada_commands;
        std::queue<ground_command>      _bbox_commands;
        std::queue<ground_command>      _cam_commands;
        std::queue<ground_command>      _arm_commands;
        std::queue<ground_command>      _netw_commands;
        /*
         * ROS Stuff
         */
        ros::NodeHandle                 _node_handle;
        /* Main Operation */
        ros::Timer                      _network_loop;

        /* Sim Clock */
        ros::Publisher                  _clock_publisher;
        rosgraph_msgs::Clock            _clock;

        /* Network Health */
        ros::Timer                      _network_health_timer;
        ros::Publisher                  _network_health_publisher;
        void networkHealth( const ros::TimerEvent &event );

        ros::Subscriber                 _simulated_command;
        ros::Subscriber                 _simulated_gtp;

        void simulatedCommandCallback( const ram_network::HaspCommand::ConstPtr &msg );
        void simulatedGTPCallback( const std_msgs::UInt32::ConstPtr &msg );



};


#endif
