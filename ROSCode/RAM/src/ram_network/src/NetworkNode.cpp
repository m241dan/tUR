//
// Created by korisd on 7/5/18.
//
#include <ram_network/NetworkNode.h>

NetworkNode::NetworkNode()
{
    setupSubscribers();
    startSerialAndI2C();
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
}
void NetworkNode::setupTimers()
{
    _network_loop           = _node_handle.createTimer( ros::Duration( refreshRate ), boost::bind( &NetworkNode::networkLoop, this, _1 ) );
    _network_health_timer   = _node_handle.createTimer( ros::Duration( healthRate ), boost::bind( &NetworkNode::networkHealth, this, _1 ) );
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

void NetworkNode::networkLoop( const ros::TimerEvent &event )
{
    handleSerial();
    handleAda();
    handleBBox();
    handleDownlink();
    _health.system_time = _registers.ard_time_sync;
    ROS_INFO( "Sync   Time: %d", _registers.gps_time_sync );
    ROS_INFO( "System Time: %d", _registers.ard_time_sync );
    ROS_INFO( "ADA    Time: %d", _registers.ada_output_register.time_register );
    ROS_INFO( "BBOX   Time: %d", _registers.bbox_output_register.time_register );
}

void NetworkNode::handleSerial()
{
    if( _handles.serial != -1 )
    {
        _health.serial_connection_fault = 0;
        // TODO below
        // check for incoming
        // parse incoming
        // handle commands
        // handle gps
        // set ada input register new_sync true
        // update register gps sync
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

void NetworkNode::handleDownlink()
{
    if( _handles.serial != -1 )
    {
        //downlinking
        downlinkPacket();
    }
}

void NetworkNode::parseSerial()
{

}

void NetworkNode::handleCommand( ground_command com )
{
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
        ROS_ERROR( "Added ADA command" );
    }
    else if( com_id >= 0x19 && com_id < 0x1E )
    {
        _bbox_commands.push( com );
    }
}

void NetworkNode::handleGTP( gtp time )
{
    std::string data( time.data );
    std::string delim( "," );
    std::string sync_time = data.substr(0, data.find(delim ));

    _registers.gps_time_sync = std::stoul( sync_time );
    _registers.ada_input_register.new_sync = 1;

}

void NetworkNode::downlinkPacket()
{

}

void NetworkNode::networkHealth( const ros::TimerEvent &event )
{
    ram_network::NetworkHealth msg;

    msg.system_time                 = _health.system_time;
    msg.serial_commands_received    = _health.serial_commands_received;
    msg.serial_gtp_received         = _health.serial_gtp_received;
    msg.serial_connection_fault     = _health.serial_connection_fault;

    msg.ada_commands_received       = _health.ada_commands_received;
    msg.ada_command_faults          = _health.ada_command_faults;
    msg.ada_writes_received         = _health.ada_writes_received;
    msg.ada_write_faults            = _health.ada_write_faults;
    msg.ada_reads_received          = _health.ada_reads_received;
    msg.ada_read_faults             = _health.ada_read_faults;
    msg.ada_sd_fault                = _health.ada_sd_fault;
    msg.ada_connection_fault        = _health.ada_connection_fault;
    msg.ada_bme01_fault             = _health.ada_bme01_fault;
    msg.ada_bme02_fault             = _health.ada_bme02_fault;
    msg.ada_eng_msg                 = std::string( _health.ada_eng_sys_msg );

    msg.bbox_commands_received      = _health.bbox_commands_received;
    msg.bbox_command_faults         = _health.bbox_command_faults;
    msg.bbox_writes_received        = _health.bbox_writes_received;
    msg.bbox_write_faults           = _health.bbox_write_faults;
    msg.bbox_reads_received         = _health.bbox_reads_received;
    msg.bbox_read_faults            = _health.bbox_read_faults;
    msg.bbox_sd_fault               = _health.bbox_sd_fault;
    msg.bbox_connection_fault       = _health.bbox_connection_fault;
    msg.bbox_eng_msg                = std::string( _health.bbox_eng_sys_msg );

    msg.ada_commands                = (uint8_t)_ada_commands.size();
    msg.bbox_commands               = (uint8_t)_bbox_commands.size();
    msg.cam_commands                = (uint8_t)_cam_commands.size();
    msg.arm_commands                = (uint8_t)_arm_commands.size();
    msg.netw_commands               = (uint8_t)_netw_commands.size();

    _network_health_publisher.publish(msg);
}

void NetworkNode::simulatedCommandCallback( const ram_network::HaspCommand::ConstPtr &msg )
{
    ground_command com;
    com.command[0] = msg->com_id;
    com.command[1] = msg->com_param;
    handleCommand( com );
    _health.serial_commands_received++;

}

void NetworkNode::simulatedGTPCallback( const std_msgs::UInt32::ConstPtr &msg )
{
    gtp time;

    snprintf( (char *)time.data, sizeof( time.data ), "%d", msg->data );
    handleGTP( time );
    _health.serial_gtp_received++;
}
