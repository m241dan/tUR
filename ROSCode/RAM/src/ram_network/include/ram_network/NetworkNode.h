//
// Created by korisd on 7/5/18.
//

#ifndef NETWORKNODE_H
#define NETWORKNODE_H

#include <ram_network/ram_network.h>
#include <queue>
#include <fstream>
#include <ram_network/Snap.h>
#include <std_srvs/Empty.h>
#include <ram_network/ManualWaypoint.h>
#include <ram_network/ServoChange.h>

class NetworkNode
{
    public:
        NetworkNode();
    protected:
        void setupSubscribers           (); // for simulating commanding
        void setupServices              (); // for commanding
        void startSerialAndI2C          ();
        void setupI2CConnections        ();
        void setupPublishers            ();
        void setupTimers                ();
        void setupComStrings            ();

        int openSerialConnection        ();
        int openAdaI2C                  ();
        int openBBoxI2C                 ();


        void networkLoop                ( const ros::TimerEvent &event );
        void handleCommand              ( ground_command &com );
        void handleGTP                  ( gtp &time );
        void handleAda                  ();
        void handleBBox                 ();
        void handleDownlink             ();
        uint8_t                         _downlink_counter;
        const uint8_t                   _downlink_when;

        void downlinkPacket             ();
        data_packet buildPacket         ();
        bool hasPackets                 ();

        void resetBuffer                ();
        NetworkHealth                   _health;
        ArduinoRegisters                _registers;
        fdHandles                       _handles;
        char                            _buffer[MAX_BUF];
        int                             _buffer_index = 0;

        bool possiblePacket             ();
        bool isCommand                  ();
        bool isGTP                      ();


        std::queue<ground_command>      _ada_commands;
        std::queue<ground_command>      _bbox_commands;
        std::queue<ground_command>      _cam_commands;
        std::queue<ground_command>      _arm_commands;
        std::queue<ground_command>      _netw_commands;

        std::queue<ambient_packet>      _ambient_packets;
        std::queue<bbox_packet>         _bbox_packets;
        std::queue<network_packet>      _network_packets;
        std::queue<image_packet>        _image_packets;
        /*
         * ROS Stuff
         */
        ros::NodeHandle                 _node_handle;
        /* Main Operation */
        ros::Timer                      _i2c_loop;
        void i2cLoopCallback            ( const ros::TimerEvent &event );

        ros::Timer                      _serial_loop;
        void serialLoopCallback         ( const ros::TimerEvent &event );

        /* Network Health */
        ros::Timer                      _network_health_timer;
        ros::Publisher                  _network_health_publisher;
        void networkHealth              ( const ros::TimerEvent &event );

        /* Sim Clock */
        ros::Publisher                  _clock_publisher;
        rosgraph_msgs::Clock            _clock;



        ros::Subscriber                 _simulated_command;
        ros::Subscriber                 _simulated_gtp;

        void simulatedCommandCallback   ( const ram_network::HaspCommand::ConstPtr &msg );
        void simulatedGTPCallback       ( const std_msgs::UInt32::ConstPtr &msg );

        ros::Timer                      _rpi_commanding;
        void rpiCommanding              ( const ros::TimerEvent &event );
        void doArmCommand               ();
        void doCamCommand               ();
        void doNetworkCommand           ();

        ros::Timer                      _register_sample;
        void registerSample             ( const ros::TimerEvent &event );
        void ambientSample              ();
        void bboxSample                 ();

        ros::Publisher                  _trial_publisher;

        void packetizeImage             ( std::string loc );
        uint16_t                        _img_counter;

        std::vector<std::string>        _cam_mons;
        ros::ServiceClient              _snap_one;
        ros::ServiceClient              _snap_two;
        ros::ServiceClient              _snap_three;
        ros::ServiceClient              _snap_four;
        ros::ServiceClient              _snap_five;
        ros::ServiceClient              _snap_six;
        ros::ServiceClient              _snap_seven;

        std::vector<std::string>        _vid_srvs;
        ros::ServiceClient              _start_vid_one;
        ros::ServiceClient              _start_vid_two;
        ros::ServiceClient              _start_vid_three;
        ros::ServiceClient              _start_vid_four;
        ros::ServiceClient              _start_vid_five;
        ros::ServiceClient              _start_vid_six;

        ros::ServiceClient              _stop_vid_one;
        ros::ServiceClient              _stop_vid_two;
        ros::ServiceClient              _stop_vid_three;
        ros::ServiceClient              _stop_vid_four;
        ros::ServiceClient              _stop_vid_five;
        ros::ServiceClient              _stop_vid_six;

        void setupManualWaypoint        ();
        ros::Publisher                  _manual_waypoint_publisher;
        ram_network::ManualWaypoint     _manual_waypoint;

        ros::Publisher                  _servo_increment_publisher;
        ros::Publisher                  _servo_decrement_publisher;
        ros::Publisher                  _arm_mode_publisher;
        ros::Publisher                  _trial_queue_reset_publisher;

};


#endif
