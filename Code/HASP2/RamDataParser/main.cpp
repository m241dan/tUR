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

    info.input_file = std::string( argv[1] );
    info.output_directory = std::string( argv[2] );

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
                                    amb_file << amb_packet.time_recorded << ",";
                                    amb_file << amb_packet.bme01_temp << ",";
                                    amb_file << amb_packet.bme01_pres << ",";
                                    amb_file << amb_packet.bme01_humi << ",";

                                    amb_file << amb_packet.bme02_temp << ",";
                                    amb_file << amb_packet.bme02_pres << ",";
                                    amb_file << amb_packet.bme02_humi << ",";

                                    amb_file << amb_packet.dallas01_temp << ",";
                                    amb_file << amb_packet.dallas02_temp << ",";
                                    amb_file << amb_packet.dallas03_temp << ",";
                                    amb_file << amb_packet.dallas04_temp << ",";
                                    amb_file << amb_packet.dallas05_temp << ",";
                                    amb_file << amb_packet.dallas06_temp << ",";
                                    amb_file << amb_packet.dallas07_temp << ",";
                                    amb_file << amb_packet.dallas08_temp << ",";
                                    amb_file << amb_packet.dallas09_temp << ",";
                                    amb_file << amb_packet.dallas10_temp << ",";
                                    amb_file << amb_packet.dallas11_temp << ",";
                                    amb_file << amb_packet.dallas12_temp << ",";
                                    amb_file << amb_packet.dallas13_temp << ",";
                                    amb_file << amb_packet.dallas14_temp << ",";
                                    amb_file << amb_packet.dallas15_temp << ",";
                                    amb_file << amb_packet.dallas16_temp << ",";
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
                                    bbox_file << b_packet.time_recorded << ",";
                                    bbox_file << b_packet.rocker_horiz << ",";
                                    bbox_file << b_packet.rocker_verti << ",";
                                    bbox_file << b_packet.toggle_horiz << ",";
                                    bbox_file << b_packet.toggle_verti << ",";
                                    bbox_file << b_packet.button_blu << ",";
                                    bbox_file << b_packet.button_blu_press_recorded << ",";
                                    bbox_file << b_packet.flap << ",";
                                    bbox_file << b_packet.potentiometer_lever << ",";
                                    bbox_file << b_packet.potentiometer_knob << ",";
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

                                    arm_file << a_packet.turntable_temp << ",";
                                    arm_file << a_packet.turntable_velo << ",";
                                    arm_file << a_packet.turntable_goal << ",";
                                    arm_file << a_packet.turntable_posi << ",";
                                    arm_file << a_packet.turntable_onoff << ",";

                                    arm_file << a_packet.shoulder_temp << ",";
                                    arm_file << a_packet.shoulder_velo << ",";
                                    arm_file << a_packet.shoulder_goal << ",";
                                    arm_file << a_packet.shoulder_posi << ",";
                                    arm_file << a_packet.shoulder_onoff << ",";

                                    arm_file << a_packet.elbow_temp << ",";
                                    arm_file << a_packet.elbow_velo << ",";
                                    arm_file << a_packet.elbow_goal << ",";
                                    arm_file << a_packet.elbow_posi << ",";
                                    arm_file << a_packet.elbow_onoff << ",";

                                    arm_file << a_packet.wrist_temp << ",";
                                    arm_file << a_packet.wrist_velo << ",";
                                    arm_file << a_packet.wrist_goal << ",";
                                    arm_file << a_packet.wrist_posi << ",";
                                    arm_file << a_packet.wrist_onoff << ",";

                                    arm_file << a_packet.wrot_temp << ",";
                                    arm_file << a_packet.wrot_velo << ",";
                                    arm_file << a_packet.wrot_goal << ",";
                                    arm_file << a_packet.wrot_posi << ",";
                                    arm_file << a_packet.wrot_onoff << ",";

                                    arm_file << a_packet.gripper_temp << ",";
                                    arm_file << a_packet.gripper_velo << ",";
                                    arm_file << a_packet.gripper_goal << ",";
                                    arm_file << a_packet.gripper_posi << ",";
                                    arm_file << a_packet.gripper_onoff << ",";
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
                                // publishMotionData( packet );
                                break;
                            }
                            case '\x36':
                            {
                                //network status
                                network_packet n_packet = extractPacket<network_packet>( packet, offset );
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