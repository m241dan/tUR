//
// Created by korisd on 7/5/18.
//
#include <ram_network/NetworkNode.h>
#include <bits/ios_base.h>

NetworkNode::NetworkNode() : _downlink_when( (uint8_t)(2.00 / serialLoop ) ), _downlink_counter(1), _img_counter(0)
{
    setupSubscribers();
    startSerialAndI2C();
    setupComStrings();
    setupServices();
    setupPublishers();
    setupTimers();
}

void NetworkNode::setupSubscribers()
{
    _simulated_command  = _node_handle.subscribe( "hasp_command", 10, &NetworkNode::simulatedCommandCallback, this );
    _simulated_gtp      = _node_handle.subscribe( "hasp_gtp", 10, &NetworkNode::simulatedGTPCallback, this );
}
void NetworkNode::setupServices()
{
    _snap_one = _node_handle.serviceClient<ram_network::Snap>( _cam_mons[0] + "/takeSnap" );
    _snap_two = _node_handle.serviceClient<ram_network::Snap>( _cam_mons[1] + "/takeSnap" );
    _snap_three = _node_handle.serviceClient<ram_network::Snap>( _cam_mons[2] + "/takeSnap" );
    _snap_four = _node_handle.serviceClient<ram_network::Snap>( _cam_mons[3] + "/takeSnap" );
    _snap_five = _node_handle.serviceClient<ram_network::Snap>( _cam_mons[4] + "/takeSnap" );
    _snap_six = _node_handle.serviceClient<ram_network::Snap>( _cam_mons[5] + "/takeSnap" );
    _snap_seven = _node_handle.serviceClient<ram_network::Snap>( _cam_mons[6] + "/takeSnap" );

}
void NetworkNode::startSerialAndI2C()
{
    _health.serial_connection_fault = openSerialConnection() == -1 ? (uint8_t )1: (uint8_t)0;
    _health.ada_connection_fault = openAdaI2C() == -1 ? (uint8_t)1 : (uint8_t)0;
    _health.bbox_connection_fault = openBBoxI2C() == -1 ? (uint8_t)1 : (uint8_t)0;
}

void NetworkNode::setupPublishers()
{
    _clock_publisher            = _node_handle.advertise<rosgraph_msgs::Clock> ( "clock", 10 );
    _network_health_publisher   = _node_handle.advertise<ram_network::NetworkHealth>( "network_health", 10 );
    _trial_publisher            = _node_handle.advertise<std_msgs::UInt8>( "trial/selector", 10 );
}
void NetworkNode::setupTimers()
{
    _i2c_loop               = _node_handle.createTimer( ros::Duration( i2cLoop ), boost::bind( &NetworkNode::i2cLoopCallback, this, _1 ) );
    _serial_loop            = _node_handle.createTimer( ros::Duration( serialLoop ), boost::bind( &NetworkNode::serialLoopCallback, this, _1 ) );
    _network_health_timer   = _node_handle.createTimer( ros::Duration( healthRate ), boost::bind( &NetworkNode::networkHealth, this, _1 ) );
    _rpi_commanding         = _node_handle.createTimer( ros::Duration( rpiComRate ), boost::bind( &NetworkNode::rpiCommanding, this, _1 ) );
    _register_sample        = _node_handle.createTimer( ros::Duration( registerRate ), boost::bind( &NetworkNode::registerSample, this, _1 ) );
}

void NetworkNode::setupComStrings()
{
    std::string mon_name;
    _node_handle.param( "mon_one", mon_name, std::string( "default" ) );
    _cam_mons.push_back( mon_name );
    _node_handle.param( "mon_two", mon_name, std::string( "default" ) );
    _cam_mons.push_back( mon_name );
    _node_handle.param( "mon_three", mon_name, std::string( "default" ) );
    _cam_mons.push_back( mon_name );
    _node_handle.param( "mon_four", mon_name, std::string( "default" ) );
    _cam_mons.push_back( mon_name );
    _node_handle.param( "mon_five", mon_name, std::string( "default" ) );
    _cam_mons.push_back( mon_name );
    _node_handle.param( "mon_six", mon_name, std::string( "default" ) );
    _cam_mons.push_back( mon_name );
    _node_handle.param( "mon_seven", mon_name, std::string( "default" ) );
    _cam_mons.push_back( mon_name );
}
int NetworkNode::openSerialConnection()
{
    if( std::getenv( "IS_RPI" ) == std::string( "true" ) ) // allow me to do more testing on laptop, remove for flight?
    {
        int attempts = 0;
        while( _handles.serial == -1 )
        {
            _handles.serial = serialOpen( serialAddress, serialBaud );
            if( _handles.serial != -1 )
            {
                ROS_INFO( "%s: Serial successfully connected", __FUNCTION__ );
            }
            else if( attempts++ > serialLimit )
            {
                ROS_ERROR( "%s: Serial failed to connect", __FUNCTION__ );
                break;
            }
        }
    }
    resetBuffer();
    return _handles.serial;
}

int NetworkNode::openAdaI2C()
{
    if( std::getenv( "IS_RPI" ) == std::string( "true" ) ) // allow me to do more testing on laptop
        _handles.ada = wiringPiI2CSetup( I2CADDRESS_ADA );

    return _handles.ada;
}

int NetworkNode::openBBoxI2C()
{
    if( std::getenv( "IS_RPI" ) == std::string( "true" ) ) // allow me to do more testing on laptop
        _handles.bbox = wiringPiI2CSetup( I2CADDRESS_BBOX );

    return _handles.bbox;
}


void NetworkNode::serialLoopCallback( const ros::TimerEvent &event )
{
    if( _handles.serial != -1 )
    {
        _health.serial_connection_fault = 0;
        if( serialDataAvail( _handles.serial ) )
        {
            ROS_INFO( "Available: %d", serialDataAvail( _handles.serial ) );
            while( serialDataAvail( _handles.serial ) )
            {
                _buffer[_buffer_index++] = (char)serialGetchar( _handles.serial );
                ROS_INFO( "Index[%d]: %c", _buffer_index-1, _buffer[_buffer_index-1] );

                if( _buffer_index > 3 )
                {
                    //remember its already been indexed by one, so have to look at least 1 back to start
                    if( possiblePacket() )
                    {
                        if( isCommand() )
                        {
                            ground_command com;
                            std::memcpy( &com, &_buffer[_buffer_index-sizeof(ground_command)], sizeof( ground_command ) );
                            handleCommand( com );
                        }
                        else if( isGTP() )
                        {
                            gtp gps;
                            std::memcpy( &gps, &_buffer[_buffer_index-sizeof(gtp)], sizeof( gtp ) );
                            handleGTP( gps );
                        }
                        else
                        {
                            resetBuffer();
                            _health.serial_bad_reads++;
                        }
                        resetBuffer();
                    }
                    else if( _buffer_index >= MAX_BUF )
                    {
                        resetBuffer();
                        _health.serial_bad_reads++;
                    }
                }
            }
        }
        if( _downlink_when <= _downlink_counter++ )
        {
            downlinkPacket();
            _downlink_counter = 1;
        }
    }
    else
    {
        if(  std::getenv( "IS_RPI" ) == std::string( "true" ) )
        {
            ROS_ERROR( "%s: we are in pure autonomous mode, attempting to reestablish connection", __FUNCTION__ );

            _health.serial_connection_fault = openSerialConnection() == -1 ? (uint8_t)1 : (uint8_t)0;
        }
    }
}

void NetworkNode::handleAda()
{
    if( _handles.ada != -1 )
    {
        _health.ada_connection_fault = 0;
        /*
         * Write Time
         */

        _registers.ada_input_register.last_write = _registers.ard_time_sync;
        _registers.ada_input_register.has_command = 0;
        _registers.ada_input_register.command_id = 0;
        _registers.ada_input_register.command_param = 0;
        if( !_ada_commands.empty() )
        {
            _registers.ada_input_register.has_command = 1;
            _registers.ada_input_register.command_id = _ada_commands.front().command[0];
            _registers.ada_input_register.command_param = _ada_commands.front().command[1];
            _ada_commands.pop();
        }
        else
        {
            _registers.ada_input_register.has_command = 0;
        }

        // write either new write or the same thing if there was a fault on the previous
        _registers.ada_input_register.setCheckSums();
        write( _handles.ada, (uint8_t *)&_registers.ada_input_register, sizeof( ADA_input_register ) );
        // TODO bad write checking?
        // read

        ADA_output_register new_read;
        read( _handles.ada, (uint8_t *)&new_read, sizeof( ADA_output_register ) );
        // TODO verify read?

        // check sums ( if bad, toss the whole read )
        //            ( if good, check for faults + update register )
        // check write fault (if write fault but checksum good, read data)
        // if not write fault, set new sync to 0
        // command faults will be handled by global reporting, TODO how do we clear?
        // update register


        if( new_read.verifyCheckSums() )
        {
            _health.ada_reads_received++;
            if( new_read.write_received )
                _health.ada_writes_received++;
            if( new_read.command_received )
                _health.ada_commands_received++;
            if( new_read.command_fault )
                _health.ada_command_faults++;

            // since ADA is the sync arduino, it has a special flag that needs to be managed
            if( new_read.write_fault )
                _health.ada_write_faults++;
            else
                _registers.ada_input_register.new_sync = 0;
            if( new_read.sd_fault )
                _health.ada_sd_fault = 1;
            if( new_read.bme01_fault )
                _health.ada_bme01_fault = 1;
            if( new_read.bme02_fault )
                _health.ada_bme02_fault = 1;
            _registers.ada_output_register = new_read;
            _clock.clock = ros::Time( (double)_registers.ada_output_register.time_register );
            _clock_publisher.publish( _clock );
        }
        else
        {
            _health.ada_read_faults++;
        }
    }
    else
    {
        /*
         * Attempt to open connection with ADA
         */
        _health.ada_connection_fault = openAdaI2C() == -1 ? (uint8_t)1 : (uint8_t)0;
    }
}

void NetworkNode::handleBBox()
{
    if( _handles.bbox != -1 )
    {
        _registers.bbox_input_register.sync_to = _registers.ard_time_sync; // always write time
        if( !_registers.bbox_output_register.write_fault )
        {
            _registers.bbox_input_register.last_write = _registers.ard_time_sync;
            //TODO commanding
        }
        _registers.bbox_input_register.setCheckSums();
        write( _handles.bbox, (uint8_t *)&_registers.bbox_input_register, sizeof( BBOX_input_register ) );

        BBOX_output_register new_read;
        read( _handles.bbox, (uint8_t *)&new_read, sizeof( BBOX_output_register ) );

        if( new_read.verifyCheckSums() )
        {
            _health.bbox_reads_received++;
            if( new_read.write_received )
                _health.bbox_writes_received++;
            if( new_read.write_fault )
                _health.bbox_write_faults++;
            if( new_read.command_fault )
                _health.bbox_command_faults++;
            if( new_read.sd_fault )
                _health.bbox_sd_fault = 1;

            _registers.bbox_output_register = new_read;
        }
        else
        {
            _health.bbox_read_faults++;
            //ROS_ERROR( "%s: BBox Check Sums bad: check_one[%d] check_two[%d], check_three[%d]", __FUNCTION__, (int)new_read.check_one, (int)new_read.check_two, (int)new_read.check_three );
            // TODO report fault somewhere
        }

    }
    else
    {
        /*
         * Attempt to open connection with BBox
         */
        _health.bbox_connection_fault = openBBoxI2C() == -1 ? (uint8_t)1 : (uint8_t)0;
    }

}

void NetworkNode::handleCommand( ground_command &com )
{
    _health.serial_commands_received++;
    uint8_t com_id = com.command[0];
    uint8_t com_args = com.command[1];

    ROS_ERROR( "com_id[%d] com_args[%d]", (int)com_id, (int)com_args );

    if( com_id >= 0x1E )
    {
        _arm_commands.push( com );
    }
    else if( com_id >= 0x0A && com_id < 0x14 )
    {
        _cam_commands.push( com );
    }
    else if( com_id >= 0x01 && com_id < 0x0A )
    {
        _netw_commands.push( com );
    }
    else if( com_id >= 0x14 && com_id < 0x19 )
    {
        _ada_commands.push( com );
    }
    else if( com_id >= 0x19 && com_id < 0x1E )
    {
        _bbox_commands.push( com );
    }
}

void NetworkNode::handleGTP( gtp &time )
{
    _health.serial_gtp_received++;
    std::string data( time.data );
    std::string delim( "." );
    std::string sync_time = data.substr(0, data.find(delim ));

    ROS_INFO( "sync time is: %s", sync_time.c_str() );
    _registers.gps_time_sync = (uint32_t)std::stoul( sync_time );
    _registers.ada_input_register.new_sync = 1;

}

void NetworkNode::downlinkPacket()
{
    data_packet data = buildPacket();
    if( data.num_data_chunks != 0 )
    {
        data.time_sent_to_HASP = _registers.ard_time_sync;
        data.setCheckSums();

        auto *ptr = (uint8_t *)&data;
        for( int x = 0; x < sizeof( data_packet ); x++ )
        {
            serialPutchar( _handles.serial, *ptr++ );
        }
    }
    /*
     * testing rig below

    data_packet data;
    memset( &data, 0, sizeof( data_packet ) );

    bbox_packet test;
    test.rocker_horiz = 1;
    test.toggle_horiz = 1;
    data.addPacket<bbox_packet>( test );

    ambient_packet amb;
    amb.bme01_temp = 100;
    amb.dallas01_temp = 1;
    amb.dallas02_temp = 2;
    amb.dallas03_temp = 3;
    data.addPacket<ambient_packet>( amb );

    data.setCheckSums();
    auto *ptr = (uint8_t *)&data;
    for( int x = 0; x < sizeof( data ); x++ )
        serialPutchar( _handles.serial, *ptr++ );
    */
}

data_packet NetworkNode::buildPacket()
{
    //check for image parts
    //check for arm packets
    //check for pathlog packets
    //check for bbox packets
    //check for ambient packets

    data_packet data;
    memset( &data, 0, sizeof( data_packet ) );
    if( hasPackets() )
    {
        while( !_image_packets.empty() )
        {
            if( data.addPacket( _image_packets.front() ) )
            {
                _image_packets.pop();
            }
            else
            {
                break;
            }
        }

        while( !_ambient_packets.empty() )
        {
            if( data.addPacket( _ambient_packets.front() ) )
            {
                _ambient_packets.pop();
            }
            else
            {
                break;
            }
        }
        while( !_bbox_packets.empty() )
        {
            if( data.addPacket( _bbox_packets.front() ) )
            {
                _bbox_packets.pop();
            }
            else
            {
                break;
            }
        }
        while( !_network_packets.empty() )
        {
            if( data.addPacket( _network_packets.front() ) )
            {
                _network_packets.pop();
            }
            {
                break;
            }

        }
    }

    return data;
}

bool NetworkNode::hasPackets()
{
    bool result = true;

    if( _ambient_packets.empty() &&
        _bbox_packets.empty() )
    {
        result = false;
    }

    return result;

}

void NetworkNode::resetBuffer()
{
    serialFlush( _handles.serial );
    std::memset( &_buffer[0], 0, MAX_BUF );
    _buffer_index = 0;
}

bool NetworkNode::possiblePacket()
{
    bool result = false;
    if( _buffer[_buffer_index - 1] == '\x0A' &&
        _buffer[_buffer_index - 2] == '\x0D' &&
        _buffer[_buffer_index - 3] == '\x03' )
    {
        result = true;
    }
    return result;
}

bool NetworkNode::isCommand()
{
    bool result = true;
    unsigned int packet_start = _buffer_index-sizeof(ground_command);
    if( _buffer[packet_start] != '\x01' ||
        _buffer[packet_start+1] != '\x02' )
    {
        result = false;
    }

    if( _buffer_index < sizeof( ground_command ) )
    {
        result = false;
    }
    return result;
}

bool NetworkNode::isGTP()
{
    bool result = true;
    unsigned int packet_start = _buffer_index-sizeof(gtp);

    if( _buffer[packet_start] != '\x01' ||
        _buffer[packet_start+1] != '\x30' )
    {
        result = false;
    }

    if( _buffer_index < sizeof( gtp ) )
    {
        result = false;
    }
    return result;
}

void NetworkNode::i2cLoopCallback( const ros::TimerEvent &event )
{
    handleAda();
    handleBBox();
    _health.system_time = _registers.ard_time_sync;
}

void NetworkNode::networkHealth( const ros::TimerEvent &event )
{
    network_packet packet;

    packet.time_recorded               = _health.system_time;
    packet.serial_commands_received    = _health.serial_commands_received;
    packet.serial_gtp_received         = _health.serial_gtp_received;
    packet.serial_bad_reads            = _health.serial_bad_reads;
    packet.serial_connection_fault     = _health.serial_connection_fault;

    packet.ada_commands_received       = _health.ada_commands_received;
    packet.ada_command_faults          = _health.ada_command_faults;
    packet.ada_writes_received         = _health.ada_writes_received;
    packet.ada_write_faults            = _health.ada_write_faults;
    packet.ada_reads_received          = _health.ada_reads_received;
    packet.ada_read_faults             = _health.ada_read_faults;
    packet.ada_sd_fault                = _health.ada_sd_fault;
    packet.ada_connection_fault        = _health.ada_connection_fault;
    packet.ada_bme01_fault             = _health.ada_bme01_fault;
    packet.ada_bme02_fault             = _health.ada_bme02_fault;
    memcpy( packet.ada_eng_sys_msg, _health.ada_eng_sys_msg, sizeof( packet.ada_eng_sys_msg ) );

    packet.bbox_commands_received      = _health.bbox_commands_received;
    packet.bbox_command_faults         = _health.bbox_command_faults;
    packet.bbox_writes_received        = _health.bbox_writes_received;
    packet.bbox_write_faults           = _health.bbox_write_faults;
    packet.bbox_reads_received         = _health.bbox_reads_received;
    packet.bbox_read_faults            = _health.bbox_read_faults;
    packet.bbox_sd_fault               = _health.bbox_sd_fault;
    packet.bbox_connection_fault       = _health.bbox_connection_fault;
    memcpy( packet.bbox_eng_sys_msg, _health.bbox_eng_sys_msg, sizeof( packet.bbox_eng_sys_msg ) );

    packet.ada_commands                = (uint8_t)_ada_commands.size();
    packet.bbox_commands               = (uint8_t)_bbox_commands.size();
    packet.cam_commands                = (uint8_t)_cam_commands.size();
    packet.arm_commands                = (uint8_t)_arm_commands.size();
    packet.netw_commands               = (uint8_t)_netw_commands.size();

    _network_packets.push( packet );
}

void NetworkNode::simulatedCommandCallback( const ram_network::HaspCommand::ConstPtr &msg )
{
    ground_command com;
    com.command[0] = msg->com_id;
    com.command[1] = msg->com_param;
    handleCommand( com );

}

void NetworkNode::simulatedGTPCallback( const std_msgs::UInt32::ConstPtr &msg )
{
    gtp time;

    snprintf( (char *)time.data, sizeof( time.data ), "%d.649$GPGGA,202212.00,3024.7205,N,09110.7264,W,1,06,1.69,00061,M,-025,M,,*51,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,", msg->data );
    handleGTP( time );
}

void NetworkNode::rpiCommanding( const ros::TimerEvent &event )
{
    /*
     * Always prioritize arm commands
     */
    if( !_arm_commands.empty() )
    {
        doArmCommand();
    }
    else if( !_cam_commands.empty() )
    {
        doCamCommand();
    }
    else if( !_netw_commands.empty() )
    {
        doNetworkCommand();
    }
}

void NetworkNode::doArmCommand()
{
    ground_command com = _arm_commands.front();
    _arm_commands.pop();

    switch( com.command[0] )
    {
        case ARM_ADD_TRIAL[0]:
            std_msgs::UInt8 trial;
            trial.data = com.command[1];
            _trial_publisher.publish(trial);
            break;
    }
}

void NetworkNode::doCamCommand()
{
    ground_command com = _cam_commands.front();
    _cam_commands.pop();
    ram_network::Snap snap;

    switch( com.command[0] )
    {
        case CAMERA_CAM_1_PIC[0]:
            if( com.command[1] == CAMERA_CAM_1_PIC[1] )
            {
               _snap_one.call(snap);
                packetizeImage( snap.response.location );
            }
            break;
        case CAMERA_CAM_2_PIC[0]:
            if( com.command[1] == CAMERA_CAM_2_PIC[1] )
            {
                _snap_two.call(snap);
                packetizeImage( snap.response.location );
            }
            break;
        case CAMERA_CAM_3_PIC[0]:
            if( com.command[1] == CAMERA_CAM_3_PIC[1] )
            {
                _snap_three.call(snap);
                packetizeImage( snap.response.location );
            }
            break;
        case CAMERA_CAM_4_PIC[0]:
            if( com.command[1] == CAMERA_CAM_4_PIC[1] )
            {
                _snap_four.call(snap);
                packetizeImage( snap.response.location );
            }
            break;
        case CAMERA_CAM_ALL_PIC[0]:
            if( com.command[2] == CAMERA_CAM_ALL_PIC[1] )
            {
                uint8_t sub = CAMERA_CAM_1_PIC[0];
                uint8_t param = CAMERA_CAM_1_PIC[1];

                for( int x = 0; x < 4; x++ )
                {
                    com.command[0] = sub++;
                    com.command[1] = param++;
                    _cam_commands.push( com );
                }
            }
            break;
        case CAMERA_RESET[0]:
            break;
        case CAMERA_VIDEO_TOGGLE[0]:
            break;
    }
}

void NetworkNode::doNetworkCommand()
{

}


void NetworkNode::registerSample( const ros::TimerEvent &event )
{
    ambientSample();
    bboxSample();
}

void NetworkNode::ambientSample()
{
    ambient_packet packet;
    ADA_output_register &register_ = _registers.ada_output_register;

    packet.time_recorded = register_.time_register;

    packet.bme01_temp = register_.bme02_temp;
    packet.bme01_pres = register_.bme02_pres;
    packet.bme01_humi = register_.bme02_humi;

    packet.bme02_temp = register_.bme02_temp;
    packet.bme02_pres = register_.bme02_pres;
    packet.bme02_humi = register_.bme02_humi;

    packet.dallas01_temp = register_.dallas01_temp;
    packet.dallas02_temp = register_.dallas02_temp;
    packet.dallas03_temp = register_.dallas03_temp;
    packet.dallas04_temp = register_.dallas04_temp;
    packet.dallas05_temp = register_.dallas05_temp;
    packet.dallas06_temp = register_.dallas06_temp;
    packet.dallas07_temp = register_.dallas07_temp;
    packet.dallas08_temp = register_.dallas08_temp;
    packet.dallas09_temp = register_.dallas09_temp;
    packet.dallas10_temp = register_.dallas10_temp;
    packet.dallas11_temp = register_.dallas11_temp;
    packet.dallas12_temp = register_.dallas12_temp;
    packet.dallas13_temp = register_.dallas13_temp;
    packet.dallas14_temp = register_.dallas14_temp;
    packet.dallas15_temp = register_.dallas15_temp;
    packet.dallas16_temp = register_.dallas16_temp;

    _ambient_packets.push( packet );
}

void NetworkNode::bboxSample()
{
    bbox_packet packet;
    BBOX_output_register &register_ = _registers.bbox_output_register;

    packet.time_recorded = register_.time_register;

    packet.rocker_horiz = register_.rocker_horiz;
    packet.rocker_verti = register_.rocker_verti;
    packet.toggle_horiz = register_.toggle_horiz;
    packet.toggle_verti = register_.toggle_verti;

    packet.button_blu = register_.button_blu;
   // TODO packet.flap = register_.flap;
    packet.potentiometer_lever = register_.potentiometer_lever;
    packet.potentiometer_knob = register_.potentiometer_knob;

    _bbox_packets.push( packet );
}

void NetworkNode::packetizeImage( std::string loc )
{
    std::ifstream img_file ( loc, std::ifstream::binary );

    if( img_file )
    {
        img_file.seekg( 0, img_file.end );
        long img_size = img_file.tellg();
        img_file.seekg( 0, img_file.beg );

        image_packet packet;
        packet.photo_number = ++_img_counter;
        while( img_size > 0 )
        {
            std::streamsize read_size = img_size > IMG_PACKET_SIZE ? IMG_PACKET_SIZE : img_size;
            packet.sizeof_photo = (uint16_t)read_size;
            img_file.read( (char *)&packet.meat[0], read_size );
            _image_packets.push( packet );
            packet.position++;
            img_size -= read_size;
        }
    }
    else
    {
        ROS_ERROR( "%s: failed to open image", __FUNCTION__ );
    }

}
