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
    _serial_input = _node_handle.advertise<std_msgs::UInt8MultiArray>( "/serial_input", 10 );

    _ambient = _node_handle.advertise<ground_station::Ambient> ("/ambient", 10 );
    _bbox = _node_handle.advertise<ground_station::BBox> ("/bbox", 10 );
    _arm_status = _node_handle.advertise<arm_motion::ArmInfo> ( "/arm_status", 10 );
    _arm_trial = _node_handle.advertise<arm_motion::TrialData> ( "/trial_data", 10 );
    _arm_motion = _node_handle.advertise<arm_motion::MotionData> ( "/motion_data", 10 );
    _network_status = _node_handle.advertise<ground_station::NetworkHealth> ( "/network_status", 10 );
    std::chrono::system_clock::time_point chrono_now = std::chrono::system_clock::now();
    time_t now = std::chrono::system_clock::to_time_t( chrono_now );
    std::stringstream ss;
    ss << std::getenv( "HOME" ) << "/log_" << now  << ".txt";
    _log = ss.str();
}

void GroundNode::timerCallback( const ros::TimerEvent &event )
{
    std_msgs::UInt8MultiArray output;
    gtp spoof;

    snprintf( spoof.data, sizeof( spoof.data ), "%f,$GPGGA,202212.00,3024.7205,N,09110.7264,W,1,06,1.69,00061,M,-025,M,,*51,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,", ros::Time::now().toSec() );

    uint8_t *ptr = (uint8_t *)&spoof;
    for( int x = 0; x < sizeof( gtp ); x++ )
        output.data.push_back( *ptr++ );

    _serial_input.publish(output);
}

void GroundNode::commandCallback( const ground_station::HaspCommand::ConstPtr &msg )
{
    std_msgs::UInt8MultiArray output;
    ground_command com;
    com.command[0] = msg->com_id;
    com.command[1] = msg->com_param;

    auto *ptr = (uint8_t *)&com;
    for( int x = 0; x < sizeof( ground_command ); x++ )
        output.data.push_back( *ptr++ );

    _serial_input.publish(output);

}

void GroundNode::outputCallback( const std_msgs::UInt8MultiArray::ConstPtr &msg )
{
    /* Grab Data and Write to File */
    std::ofstream output_file;
    output_file.open( _log, std::ofstream::app );
    for( auto val : msg->data )
    {
        _buffer.push_back( val );
        output_file <<  val;
    }
    output_file.close();

    if( _buffer.size() == sizeof( data_packet ) )
    {
        data_packet data;
        auto *ptr = (uint8_t *)&data;
        for( auto val : _buffer )
        {
            *ptr++ = val;
        }
        if( data.verifyCheckSums() )
        {
            ROS_ERROR( "Check Sums Validated!" );
            uint16_t offset = 0;
            for( int x = 0; x < data.num_data_chunks; x++ )
            {
                switch( data.meat[offset] )
                {
                    case '\x30':
                    {
                        image_packet packet = extractPacket<image_packet>( data, offset );
                        std::stringstream ss;
                        ss << "/home/korisd/test" << packet.photo_number << ".png";
                        std::ofstream img_file( ss.str(), std::ofstream::binary | std::ofstream::app );
                        if( img_file )
                        {
                            img_file.write( (char *)&packet.meat, packet.sizeof_photo );
                        }
                        else
                        {
                            ROS_ERROR( "%s: failed to open img", __FUNCTION__ );
                        }
                        break;
                    }
                    case '\x31':
                    {
                        //ambient
                        ambient_packet packet = extractPacket<ambient_packet>( data, offset );
                        publishAmbient( packet );
                        break;
                    }
                    case '\x32':
                    {
                        //bbox
                        bbox_packet packet = extractPacket<bbox_packet>( data, offset );
                        publishBBox( packet );
                        break;
                    }
                    case '\x33':
                    {
                        //arm_packet
                        arm_packet packet = extractPacket<arm_packet>( data, offset );
                        publishArmStatus( packet );
                        break;
                    }
                    case '\x35':
                    {
                        //trial pack
                        trial_packet packet = extractPacket<trial_packet>( data, offset );
                        publishTrialData( packet );
                        break;
                    }
                    case '\x37':
                    {
                        motion_packet packet = extractPacket<motion_packet>( data, offset );
                        publishMotionData( packet );
                        break;
                    }
                    case '\x36':
                    {
                        //network status
                        network_packet packet = extractPacket<network_packet>( data, offset );
                        publishNetworkStatus( packet );
                        break;
                    }
                }
            }
        }
        else
        {
            ROS_ERROR( "Bad Checksums!" );
        }
        ROS_ERROR( "Clearing Buffer" );
        _buffer.clear();

    }
    else if( _buffer.size() > sizeof( data_packet ) )
    {
        _buffer.clear();
        ROS_ERROR( "There's been buffer overflow!!!" );
    }

}

template<typename packet_type>
packet_type GroundNode::extractPacket( data_packet &data, uint16_t &offset )
{
    packet_type packet;
    memcpy( &packet, &data.meat[offset], sizeof( packet_type ) );
    offset += sizeof( packet_type );
    return packet;
}

void GroundNode::publishAmbient( ambient_packet &packet )
{
    ground_station::Ambient msg;

    msg.time_recorded = packet.time_recorded;

    msg.bme01_temp = packet.bme01_temp;
    msg.bme01_pres = packet.bme01_pres;
    msg.bme01_humi = packet.bme01_humi;

    msg.bme02_temp = packet.bme02_temp;
    msg.bme02_pres = packet.bme02_pres;
    msg.bme02_humi = packet.bme02_humi;

    msg.dallas01_temp = packet.dallas01_temp;
    msg.dallas02_temp = packet.dallas02_temp;
    msg.dallas03_temp = packet.dallas03_temp;
    msg.dallas04_temp = packet.dallas04_temp;
    msg.dallas05_temp = packet.dallas05_temp;
    msg.dallas06_temp = packet.dallas06_temp;
    msg.dallas07_temp = packet.dallas07_temp;
    msg.dallas08_temp = packet.dallas08_temp;
    msg.dallas09_temp = packet.dallas09_temp;
    msg.dallas10_temp = packet.dallas10_temp;
    msg.dallas11_temp = packet.dallas11_temp;
    msg.dallas12_temp = packet.dallas12_temp;
    msg.dallas13_temp = packet.dallas13_temp;
    msg.dallas14_temp = packet.dallas14_temp;
    msg.dallas15_temp = packet.dallas15_temp;
    msg.dallas16_temp = packet.dallas16_temp;

    _ambient.publish(msg);
}

void GroundNode::publishBBox( bbox_packet &packet )
{
    ground_station::BBox msg;

    msg.time_recorded = packet.time_recorded;

    msg.rocker_horiz = packet.rocker_horiz;
    msg.rocker_verti = packet.rocker_verti;
    msg.toggle_horiz = packet.toggle_horiz;
    msg.toggle_verti = packet.toggle_verti;

    msg.button_blu = packet.button_blu;
    msg.potentiometer_lever = packet.potentiometer_lever;
    msg.potentiometer_knob = packet.potentiometer_knob;

    _bbox.publish(msg);
}

void GroundNode::publishArmStatus( arm_packet &packet )
{
    arm_motion::ArmInfo msg;

    dynamixel_workbench_msgs::XH servo;
    servo.Torque_Enable = (uint8_t)packet.turntable_onoff;
    servo.Present_Position = packet.turntable_posi;
    servo.Present_Velocity = packet.turntable_velo;
    servo.Goal_Position = (uint32_t )packet.turntable_goal;
    servo.Present_Temperature = (uint8_t)packet.turntable_temp;
    msg.servos.push_back( servo );

    servo.Torque_Enable = (uint8_t)packet.shoulder_onoff;
    servo.Present_Position = packet.shoulder_posi;
    servo.Present_Velocity = packet.shoulder_velo;
    servo.Goal_Position = (uint32_t )packet.shoulder_goal;
    servo.Present_Temperature = (uint8_t)packet.shoulder_temp;
    msg.servos.push_back( servo );

    servo.Torque_Enable = (uint8_t)packet.elbow_onoff;
    servo.Present_Position = packet.elbow_posi;
    servo.Present_Velocity = packet.elbow_velo;
    servo.Goal_Position = (uint32_t )packet.elbow_goal;
    servo.Present_Temperature = (uint8_t)packet.elbow_temp;
    msg.servos.push_back( servo );

    servo.Torque_Enable = (uint8_t)packet.wrist_onoff;
    servo.Present_Position = packet.wrist_posi;
    servo.Present_Velocity = packet.wrist_velo;
    servo.Goal_Position = (uint32_t )packet.wrist_goal;
    servo.Present_Temperature = (uint8_t)packet.wrist_temp;
    msg.servos.push_back( servo );

    servo.Torque_Enable = (uint8_t)packet.wrot_onoff;
    servo.Present_Position = packet.wrot_posi;
    servo.Present_Velocity = packet.wrot_velo;
    servo.Goal_Position = (uint32_t )packet.wrot_goal;
    servo.Present_Temperature = (uint8_t)packet.wrot_temp;
    msg.servos.push_back( servo );

    servo.Torque_Enable = (uint8_t)packet.gripper_onoff;
    servo.Present_Position = packet.gripper_posi;
    servo.Present_Velocity = packet.gripper_velo;
    servo.Goal_Position = (uint32_t )packet.gripper_goal;
    servo.Present_Temperature = (uint8_t)packet.gripper_temp;
    msg.servos.push_back( servo );


    _arm_status.publish( msg );
}

void GroundNode::publishTrialData( trial_packet &packet )
{
    arm_motion::TrialData msg;

    msg.trial_name = std::string( packet.trial_name );
    msg.start_time = packet.trial_time_start;
    msg.stop_time = packet.trial_time_end;

    _arm_trial.publish( msg );
}


void GroundNode::publishMotionData( motion_packet &packet )
{
    arm_motion::MotionData msg;

    msg.start_time = packet.start_time;
    msg.stop_time = packet.stop_time;
    msg.servo_one_start_pos = packet.joint_one_start;
    msg.servo_two_start_pos = packet.joint_two_start;
    msg.servo_three_start_pos = packet.joint_three_start;
    msg.servo_four_start_pos = packet.joint_four_start;
    msg.servo_five_start_pos = packet.joint_five_start;
    msg.servo_six_start_pos = packet.joint_six_start;

    msg.servo_one_stop_pos = packet.joint_one_stop;
    msg.servo_two_stop_pos = packet.joint_two_stop;
    msg.servo_three_stop_pos = packet.joint_three_stop;
    msg.servo_four_stop_pos = packet.joint_four_stop;
    msg.servo_five_stop_pos = packet.joint_five_stop;
    msg.servo_six_stop_pos = packet.joint_six_stop;

    msg.start_x = packet.start_x;
    msg.start_y = packet.start_y;
    msg.start_z = packet.start_z;
    msg.start_e = packet.start_e;

    msg.stop_x = packet.stop_x;
    msg.stop_y = packet.stop_y;
    msg.stop_z = packet.stop_z;
    msg.stop_e = packet.stop_e;


}
void GroundNode::publishNetworkStatus( network_packet &packet )
{
    ground_station::NetworkHealth msg;

    msg.system_time                 = packet.time_recorded;
    msg.serial_commands_received    = packet.serial_commands_received;
    msg.serial_gtp_received         = packet.serial_gtp_received;
    msg.serial_bad_reads            = packet.serial_bad_reads;
    msg.serial_connection_fault     = packet.serial_connection_fault;

    msg.ada_commands_received       = packet.ada_commands_received;
    msg.ada_command_faults          = packet.ada_command_faults;
    msg.ada_writes_received         = packet.ada_writes_received;
    msg.ada_write_faults            = packet.ada_write_faults;
    msg.ada_reads_received          = packet.ada_reads_received;
    msg.ada_read_faults             = packet.ada_read_faults;
    msg.ada_sd_fault                = packet.ada_sd_fault;
    msg.ada_connection_fault        = packet.ada_connection_fault;
    msg.ada_bme01_fault             = packet.ada_bme01_fault;
    msg.ada_bme02_fault             = packet.ada_bme02_fault;
    msg.ada_eng_msg                 = std::string( packet.ada_eng_sys_msg );

    msg.bbox_commands_received      = packet.bbox_commands_received;
    msg.bbox_command_faults         = packet.bbox_command_faults;
    msg.bbox_writes_received        = packet.bbox_writes_received;
    msg.bbox_write_faults           = packet.bbox_write_faults;
    msg.bbox_reads_received         = packet.bbox_reads_received;
    msg.bbox_read_faults            = packet.bbox_read_faults;
    msg.bbox_sd_fault               = packet.bbox_sd_fault;
    msg.bbox_connection_fault       = packet.bbox_connection_fault;
    msg.bbox_eng_msg                = std::string( packet.bbox_eng_sys_msg );

    msg.ada_commands                = packet.ada_commands;
    msg.bbox_commands               = packet.bbox_commands;
    msg.cam_commands                = packet.cam_commands;
    msg.arm_commands                = packet.arm_commands;
    msg.netw_commands               = packet.netw_commands;

    _network_status.publish(msg);
}
