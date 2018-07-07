//
// Created by korisd on 7/5/18.
//
#include <ram_network/NetworkNode.h>

NetworkNode::NetworkNode()
{
    setupSubscribers();
    setupSerialConnection();
    setupI2CConnections();
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

void NetworkNode::setupSerialConnection()
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
}

void NetworkNode::setupI2CConnections()
{
    openAdaI2C();
    openBBoxI2C();
}

void NetworkNode::setupPublishers()
{
    _clock_publisher = _node_handle.advertise<rosgraph_msgs::Clock> ( "clock", 10 );
}
void NetworkNode::setupTimers()
{
    _network_loop = _node_handle.createTimer( ros::Duration( refreshRate ), boost::bind( &NetworkNode::networkLoop, this, _1 ) );
}

void NetworkNode::openAdaI2C()
{
    if( std::getenv( "IS_RPI" ) == std::string( "true" ) ) // allow me to do more testing on laptop
        _handles.ada = wiringPiI2CSetup( I2CADDRESS_ADA );
}

void NetworkNode::openBBoxI2C()
{
    if( std::getenv( "IS_RPI" ) == std::string( "true" ) ) // allow me to do more testing on laptop
        _handles.bbox = wiringPiI2CSetup( I2CADDRESS_BBOX );
}

void NetworkNode::networkLoop( const ros::TimerEvent &event )
{
    if( _handles.serial != -1 )
    {
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
            setupSerialConnection();
        }
    }
    if( _handles.ada != -1 )
    {
        /*
         * Write Time
         */
        if( _registers.ada_output_register.write_fault )
        {
            _registers.ada_input_register.last_write    = _registers.ard_time_sync;
            _registers.ada_input_register.has_command   = 0;
            _registers.ada_input_register.command_id    = 0;
            _registers.ada_input_register.command_param = 0;
            if( !_ada_commands.empty() )
            {
                _registers.ada_input_register.has_command = 1;
                _registers.ada_input_register.command_id = _ada_commands.front().command[0];
                _registers.ada_input_register.command_param = _ada_commands.front().command[1];
                //TODO if command fault is set, don't send any commands (let that be cleared by ground) (I THINK THIS WILL JUST A COUNTER AND NO RESETTING)
            }
            else
            {
                _registers.ada_input_register.has_command   = 0;
            }
        }
        // write either new write or the same thing if there was a fault on the previous
        _registers.ada_input_register.setCheckSums();
        write( _handles.ada, (byte *)&_registers.ada_input_register, sizeof( ADA_input_register ) );
        // TODO bad write checking?
        // read

        ADA_output_register new_read;
        read( _handles.ada, (byte *)&new_read, sizeof( ADA_output_register ) );
        // TODO verify read?

        // check sums ( if bad, toss the whole read )
        //            ( if good, check for faults + update register )
        // check write fault (if write fault but checksum good, read data)
          // if not write fault, set new sync to 0
        // command faults will be handled by global reporting, TODO how do we clear?
        // update register


        if( new_read.verifyCheckSums() )
        {
            if( !new_read.write_fault )
                _registers.ada_input_register.new_sync = 0;
            _registers.ada_output_register = new_read;
            _clock.clock = ros::Time( (double)_registers.ada_output_register.time_register );
            _clock_publisher.publish( _clock );
        }
        else
        {
            // TODO report fault somewhere
        }
    }
    else
    {
        /*
         * Attempt to open connection with ADA
         */
        openAdaI2C();
    }

    if( _handles.bbox != -1 )
    {
        _registers.bbox_input_register.sync_to = _registers.ard_time_sync; // always write time
        if( !_registers.bbox_output_register.write_fault )
        {
            _registers.bbox_input_register.last_write = _registers.ard_time_sync;
            //TODO commanding
        }
        _registers.bbox_input_register.setCheckSums();
        write( _handles.bbox, (byte *)&_registers.bbox_input_register, sizeof( BBOX_input_register ) );

        BBOX_output_register new_read;
        read( _handles.bbox, (byte *)&new_read, sizeof( BBOX_output_register ) );

        if( new_read.verifyCheckSums() )
        {
            _registers.bbox_output_register = new_read;
        }
        else
        {
            //ROS_ERROR( "%s: BBox Check Sums bad: check_one[%d] check_two[%d], check_three[%d]", __FUNCTION__, (int)new_read.check_one, (int)new_read.check_two, (int)new_read.check_three );
            // TODO report fault somewhere
        }

    }
    else
    {
        /*
         * Attempt to open connection with BBox
         */
        openBBoxI2C();
    }

    if( _handles.serial != -1 )
    {
        //downlinking
        downlinkPacket();
    }
    ROS_INFO( "Sync   Time: %lu", _registers.gps_time_sync );
    ROS_INFO( "System Time: %lu", _registers.ard_time_sync );
    ROS_INFO( "ADA    Time: %lu", _registers.ada_output_register.time_register );
    ROS_INFO( "BBOX   Time: %lu", _registers.bbox_output_register.time_register );
}

void NetworkNode::handleSerial()
{

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

void NetworkNode::simulatedCommandCallback( const std_msgs::UInt8MultiArray::ConstPtr &msg )
{
    ground_command com;
    com.command[0] = msg->data[0];
    com.command[1] = msg->data[1];
    handleCommand( com );

}

void NetworkNode::simulatedGTPCallback( const std_msgs::UInt32::ConstPtr &msg )
{
    gtp time;

    snprintf( (char *)time.data, sizeof( time.data ), "%d", msg->data );
    handleGTP( time );
}
