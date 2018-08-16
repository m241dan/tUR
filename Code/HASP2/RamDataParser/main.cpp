#include <iostream>
#include <vector>
#include <ram_funcs.h>
#include <fstream>
#include <memory>
#include <cstring>
#include <sstream>

struct files {
    std::string input_file;
    std::string output_directory;
};

files parse_args( int argc, char *argv[] )
{
    files info;

    info.output_directory = std::string( argv[1] );
    info.input_file = std::string( argv[2] );

    return info;
}

template<typename packet_type>
packet_type extractPacket( data_packet &data, long &offset )
{
    packet_type packet;
    memcpy( &packet, &data.meat[offset], sizeof( packet_type ) );
    offset += sizeof( packet_type );
    return packet;
}

int main( int argc, char *argv[] )
{
    files info = parse_args( argc, argv );

    std::vector<char> buffer;
    std::ifstream input_file( info.input_file, std::ifstream::binary );

    if( input_file )
    {
        input_file.seekg( 0, input_file.end );
        long input_size = input_file.tellg();
        input_file.seekg( 0, input_file.beg );

        auto *buf = new char;
        for( long x = 0; x < input_size; x++ )
        {
            input_file.read( buf, 1 );
            buffer.push_back( *buf );
        }

        //parse the input
        for( long s = 0; s < buffer.size(); s++ )
        {
            if( buffer[s] == '\x01' &&
                buffer[s+1] == '\x21' )
            {
                data_packet packet;
                memcpy( &packet, buffer.data()+s, sizeof( data_packet ) );
                //std::cout << packet.verifyCheckSums() << std::endl;
                if( packet.verifyCheckSums() )
                {
                    long offset = 0;
                    for( int x = 0; x < packet.num_data_chunks; x++ )
                    {
                        switch( packet.meat[offset] )
                        {
                            case '\x30':
                            {
                                image_packet img_packet = extractPacket<image_packet>( packet, offset );
                                std::stringstream ss;
                                ss << info.output_directory << "/img" << img_packet.photo_number << ".png";
                                std::ofstream img_file( ss.str(), std::ofstream::binary | std::ofstream::app );
                                if( img_file )
                                {
                                    img_file.write((char *) &img_packet.meat, img_packet.sizeof_photo );
                                }
                                else
                                {
                                    std::cout << __FUNCTION__ << ": failed to open img" << std::endl;
                                }
                                break;
                            }
                            case '\x31':
                            {
                                //ambient
                                ambient_packet amb_packet = extractPacket<ambient_packet>( packet, offset );
                                std::stringstream ss;
                                ss << info.output_directory << "/ambient.csv";
                                std::ofstream amb_file( ss.str(), std::ofstream::app );
                                if( amb_file )
                                {
                                    amb_file << (int)amb_packet.time_recorded << ",";

                                    amb_file << (int)amb_packet.bme01_temp << ",";                                  
									//DATA SANITIZATION to discard bullshit outliers
									if ( (int)amb_packet.bme01_pres < -1000) 
									{
										amb_file << "bad,";
									}
									else
									{
                                        amb_file << (int)amb_packet.bme01_pres << ",";
									}
									amb_file << (int)amb_packet.bme01_humi << ",";

                                    amb_file << (int)amb_packet.bme02_temp << ",";
                                    if ( (int)amb_packet.bme02_pres < -1000) 
									{
										amb_file << "bad,";
									}
									else
									{
                                        amb_file << (int)amb_packet.bme02_pres << ",";
									}
                                    amb_file << (int)amb_packet.bme02_humi << ",";

									/*
									if ( ((double)amb_packet.dallas00_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas00_temp / 100.0 << ",";
									}
									*/
									
                                    if ( ((double)amb_packet.dallas01_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas01_temp / 100.0 << ",";
									}
									
                                    if ( ((double)amb_packet.dallas02_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas02_temp / 100.0 << ",";
									}
									
									if ( ((double)amb_packet.dallas03_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas03_temp / 100.0 << ",";
									}
                                    
                                    if ( ((double)amb_packet.dallas04_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas04_temp / 100.0 << ",";
									}
									
                                    if ( ((double)amb_packet.dallas05_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas05_temp / 100.0 << ",";
									}
									
                                    if ( ((double)amb_packet.dallas06_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas06_temp / 100.0 << ",";
									}
									
                                    if ( ((double)amb_packet.dallas07_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas07_temp / 100.0 << ",";
									}
									
                                    if ( ((double)amb_packet.dallas08_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas08_temp / 100.0 << ",";
									}
									
                                    if ( ((double)amb_packet.dallas09_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas09_temp / 100.0 << ",";
									}
									
                                    if ( ((double)amb_packet.dallas10_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas10_temp / 100.0 << ",";
									}
									
                                    if ( ((double)amb_packet.dallas11_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas11_temp / 100.0 << ",";
									}
									
                                    if ( ((double)amb_packet.dallas12_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas12_temp / 100.0 << ",";
									}
									
                                    if ( ((double)amb_packet.dallas13_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas13_temp / 100.0 << ",";
									}
									
                                    if ( ((double)amb_packet.dallas14_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas14_temp / 100.0 << ",";
									}
									
                                    if ( ((double)amb_packet.dallas15_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas15_temp / 100.0 << ",";
									}
									
                                    if ( ((double)amb_packet.dallas16_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas16_temp / 100.0 << ",";
									}
									
									if ( ((double)amb_packet.dallas17_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas17_temp / 100.0 << ",";
									}
									
									if ( ((double)amb_packet.dallas18_temp / 100.0) < -200.0)
									{
										amb_file << "bad,";
									}
									else
									{
										amb_file << (double)amb_packet.dallas18_temp / 100.0 << ",";
									}									
                                    amb_file << std::endl;


                                }
                                else
                                {
                                    std::cout << __FUNCTION__ << ": cannot open amb file" << std::endl;
                                }

                                //publishAmbient( packet );
                                break;
                            }
                            case '\x32':
                            {
                                //bbox
                                bbox_packet b_packet = extractPacket<bbox_packet>( packet, offset );
                                std::stringstream ss;
                                ss << info.output_directory << "/bbox.csv";
                                std::ofstream bbox_file( ss.str(), std::ofstream::app );
                                if( bbox_file )
                                {
                                    bbox_file << (int)b_packet.time_recorded << ",";
                                    bbox_file << (int)b_packet.rocker_horiz << ",";
                                    bbox_file << (int)b_packet.rocker_verti << ",";
                                    bbox_file << (int)b_packet.toggle_horiz << ",";
                                    bbox_file << (int)b_packet.toggle_verti << ",";
                                    bbox_file << (int)b_packet.button_blu << ",";
                                    bbox_file << (int)b_packet.flap << ",";
                                    bbox_file << (int)b_packet.potentiometer_lever << ",";
                                    bbox_file << (int)b_packet.potentiometer_knob << ",";
                                    bbox_file << std::endl;
                                }
                                else
                                {
                                    std::cout << __FUNCTION__ << ": cannot open bbox file" << std::endl;
                                }

                                //publishBBox( packet );
                                break;
                            }
                            case '\x33':
                            {
                                //arm_packet
                                arm_packet a_packet = extractPacket<arm_packet>( packet, offset );
                                std::stringstream ss;
                                ss << info.output_directory << "/arm.csv";
                                std::ofstream arm_file( ss.str(), std::ofstream::app );
                                if( arm_file )
                                {
                                    arm_file << a_packet.time_recorded << ",";

                                    arm_file << (int)a_packet.turntable_temp << ",";
                                    arm_file << (int)a_packet.turntable_velo << ",";
                                    arm_file << (int)a_packet.turntable_goal << ",";
                                    arm_file << (int)a_packet.turntable_posi << ",";
                                    arm_file << (int)a_packet.turntable_onoff << ",";

                                    arm_file << (int)a_packet.shoulder_temp << ",";
                                    arm_file << (int)a_packet.shoulder_velo << ",";
                                    arm_file << (int)a_packet.shoulder_goal << ",";
                                    arm_file << (int)a_packet.shoulder_posi << ",";
                                    arm_file << (int)a_packet.shoulder_onoff << ",";

                                    arm_file << (int)a_packet.elbow_temp << ",";
                                    arm_file << (int)a_packet.elbow_velo << ",";
                                    arm_file << (int)a_packet.elbow_goal << ",";
                                    arm_file << (int)a_packet.elbow_posi << ",";
                                    arm_file << (int)a_packet.elbow_onoff << ",";

                                    arm_file << (int)a_packet.wrist_temp << ",";
                                    arm_file << (int)a_packet.wrist_velo << ",";
                                    arm_file << (int)a_packet.wrist_goal << ",";
                                    arm_file << (int)a_packet.wrist_posi << ",";
                                    arm_file << (int)a_packet.wrist_onoff << ",";

                                    arm_file << (int)a_packet.wrot_temp << ",";
                                    arm_file << (int)a_packet.wrot_velo << ",";
                                    arm_file << (int)a_packet.wrot_goal << ",";
                                    arm_file << (int)a_packet.wrot_posi << ",";
                                    arm_file << (int)a_packet.wrot_onoff << ",";

                                    arm_file << (int)a_packet.gripper_temp << ",";
                                    arm_file << (int)a_packet.gripper_velo << ",";
                                    arm_file << (int)a_packet.gripper_goal << ",";
                                    arm_file << (int)a_packet.gripper_posi << ",";
                                    arm_file << (int)a_packet.gripper_onoff << ",";
                                    arm_file << std::endl;
                                }
                                else
                                {
                                    std::cout << __FUNCTION__ << ": cannot open arm file " << std::endl;
                                }
                                // publishArmStatus( packet );
                                break;
                            }
                            case '\x35':
                            {
                                //trial pack
                                trial_packet t_packet = extractPacket<trial_packet>( packet, offset );
                                std::stringstream ss;
                                ss << info.output_directory << "/trial.csv";
                                std::ofstream trial_file( ss.str(), std::ofstream::app );
                                if( trial_file )
                                {
                                    trial_file << t_packet.trial_name << ",";
                                    trial_file << (int)t_packet.trial_time_start << ",";
                                    trial_file << (int)t_packet.trial_time_end << ",";
                                    trial_file << std::endl;
                                }
                                else
                                {
                                    std::cout << __FUNCTION__ << ": cannot open trial file" << std::endl;
                                }
                                // publishTrialData( packet );
                                break;
                            }
                            case '\x37':
                            {
                                motion_packet m_packet = extractPacket<motion_packet>( packet, offset );
                                std::stringstream ss;
                                ss << info.output_directory << "/motion.csv";
                                std::ofstream motion_file( ss.str(), std::ofstream::app );
                                if( motion_file )
                                {
                                    motion_file << (int)m_packet.start_time << ",";
                                    motion_file << (int)m_packet.stop_time << ",";

                                    motion_file << m_packet.start_x << ",";
                                    motion_file << m_packet.start_y << ",";
                                    motion_file << m_packet.start_z << ",";
                                    motion_file << m_packet.start_e << ",";

                                    motion_file << m_packet.stop_x << ",";
                                    motion_file << m_packet.stop_y << ",";
                                    motion_file << m_packet.stop_z << ",";
                                    motion_file << m_packet.stop_e << ",";

                                    motion_file << (int)m_packet.joint_one_start << ",";
                                    motion_file << (int)m_packet.joint_one_stop << ",";
                                    motion_file << (int)m_packet.joint_two_start << ",";
                                    motion_file << (int)m_packet.joint_two_stop << ",";
                                    motion_file << (int)m_packet.joint_three_start << ",";
                                    motion_file << (int)m_packet.joint_three_stop << ",";
                                    motion_file << (int)m_packet.joint_four_start << ",";
                                    motion_file << (int)m_packet.joint_four_stop << ",";
                                    motion_file << (int)m_packet.joint_five_start << ",";
                                    motion_file << (int)m_packet.joint_five_stop << ",";
                                    motion_file << (int)m_packet.joint_six_start << ",";
                                    motion_file << (int)m_packet.joint_six_stop << ",";
                                    motion_file << std::endl;

                                }
                                else
                                {
                                    std::cout << __FUNCTION__ << ": cannot open motion file" << std::endl;
                                }
                                // publishMotionData( packet );
                                break;
                            }
                            case '\x36':
                            {
                                //network status
                                network_packet n_packet = extractPacket<network_packet>( packet, offset );
                                std::stringstream ss;
                                ss << info.output_directory << "network.csv";
                                std::ofstream network_file( ss.str(), std::ofstream::app );
                                if( network_file )
                                {
                                    network_file << (int)n_packet.time_recorded << ",";
                                    network_file << (int)n_packet.serial_commands_received << ",";
                                    network_file << (int)n_packet.serial_gtp_received << ",";
                                    network_file << (int)n_packet.serial_bad_reads << ",";
                                    network_file << (int)n_packet.serial_connection_fault << ",";

                                    network_file << (int)n_packet.ada_commands_received << ",";
                                    network_file << (int)n_packet.ada_command_faults << ",";
                                    network_file << (int)n_packet.ada_writes_received << ",";
                                    network_file << (int)n_packet.ada_write_faults << ",";
                                    network_file << (int)n_packet.ada_reads_received << ",";
                                    network_file << (int)n_packet.ada_read_faults << ",";
                                    network_file << (int)n_packet.ada_sd_fault << ",";
                                    network_file << (int)n_packet.ada_connection_fault << ",";
                                    network_file << (int)n_packet.ada_bme01_fault << ",";
                                    network_file << (int)n_packet.ada_bme02_fault << ",";
                                    network_file << n_packet.ada_eng_sys_msg << ",";

                                    network_file << (int)n_packet.bbox_commands_received << ",";
                                    network_file << (int)n_packet.bbox_command_faults << ",";
                                    network_file << (int)n_packet.bbox_writes_received << ",";
                                    network_file << (int)n_packet.bbox_write_faults << ",";
                                    network_file << (int)n_packet.bbox_reads_received << ",";
                                    network_file << (int)n_packet.bbox_read_faults << ",";
                                    network_file << (int)n_packet.bbox_sd_fault << ",";
                                    network_file << (int)n_packet.bbox_connection_fault << ",";
                                    network_file << n_packet.bbox_eng_sys_msg << ",";

                                    network_file << (int)n_packet.ada_commands << ",";
                                    network_file << (int)n_packet.bbox_commands << ",";
                                    network_file << (int)n_packet.cam_commands << ",";
                                    network_file << (int)n_packet.arm_commands << ",";
                                    network_file << (int)n_packet.netw_commands << ",";
                                    network_file << std::endl;
                                }
                                else
                                {
                                    std::cout << __FUNCTION__ << ": cannot open network file" << std::endl;
                                }
                                // publishNetworkStatus( packet );
                                break;
                            }
                        }
                    }
                }

            }
        }

    }
    else
    {
        std::cout << "Input file was not opened." << std::endl;
    }



    return 0;
}
