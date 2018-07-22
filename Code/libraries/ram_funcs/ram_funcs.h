#ifndef ram_funcs_h
#define ram_funcs_h

#include "time.h"
#if ARDUINO_ARCH_SAMD

#include "Arduino.h"
#include "SD.h"

String getNextFile( String name, String append );

#endif

const uint8_t DEADZONE_WIDTH = 1; //By how many units (range 0-100) does the potentiometer ignore noise. 2 may be a touch high; consider nudging down to 1 later. -JA

// I2C ADDRESSING
// Arduino pin 4 (the data, or SDA, pin) is WHT wire
// Arduino pin 5 (the clock, or SCL, pin) is YEL wire
const uint8_t I2CADDRESS_BBOX = 0x06;
const uint8_t I2CADDRESS_ADA  = 0x04;
const uint16_t MEAT_SIZE = 468;
const uint8_t CHECKSUMZ = 28;
const uint16_t PACKET_SIZE = 512;
const uint16_t IMG_PACKET_SIZE = 462;


struct image_packet
{
    uint8_t header = '\x30';
    uint8_t position = 0;
    uint16_t photo_number = 0;
    uint16_t sizeof_photo = 0;
    uint8_t meat[IMG_PACKET_SIZE] = { 0 };
};

struct ambient_packet
{
    uint8_t header = '\x31';
    uint32_t time_recorded = 0;

    int16_t bme01_temp = 0;
    int16_t bme01_pres = 0;
    uint8_t bme01_humi = 0;

    int16_t bme02_temp = 0;
    int16_t bme02_pres = 0;
    uint8_t bme02_humi = 0;

    int16_t dallas01_temp = 0;
    int16_t dallas02_temp = 0;
    int16_t dallas03_temp = 0;
    int16_t dallas04_temp = 0;
    int16_t dallas05_temp = 0;
    int16_t dallas06_temp = 0;
    int16_t dallas07_temp = 0;
    int16_t dallas08_temp = 0;
    int16_t dallas09_temp = 0;
    int16_t dallas10_temp = 0;
    int16_t dallas11_temp = 0;
    int16_t dallas12_temp = 0;
    int16_t dallas13_temp = 0;
    int16_t dallas14_temp = 0;
    int16_t dallas15_temp = 0;
    int16_t dallas16_temp = 0;
};

struct bbox_packet
{
    uint8_t header = '\x32';
    uint32_t time_recorded = 0;

    uint8_t rocker_horiz = 0;
    uint8_t rocker_verti = 0;
    uint8_t toggle_horiz = 0;
    uint8_t toggle_verti = 0;
    uint8_t button_blu = 0;
    uint32_t button_blu_press_recorded = 0;
    uint8_t flap = 0;
    uint8_t potentiometer_lever = 0;
    uint8_t potentiometer_knob = 0;
};

struct arm_packet
{
    uint8_t header = '\x33';
    uint32_t time_recorded = 0;

    int8_t turntable_temp = 0;
    uint8_t turntable_velo = 0;
    int16_t turntable_goal = 0;
    int16_t turntable_posi = 0;
    bool turntable_onoff = false;

    int8_t shoulder_temp = 0;
    uint8_t shoulder_velo = 0;
    int16_t shoulder_goal = 0;
    int16_t shoulder_posi = 0;
    bool shoulder_onoff = false;

    int8_t elbow_temp = 0;
    uint8_t elbow_velo = 0;
    int16_t elbow_goal = 0;
    int16_t elbow_posi = 0;
    bool elbow_onoff = false;

    int8_t wrist_temp = 0;
    uint8_t wrist_velo = 0;
    int16_t wrist_goal = 0;
    int16_t wrist_posi = 0;
    bool wrist_onoff = false;

    int8_t wrot_temp = 0;
    uint8_t wrot_velo = 0;
    int16_t wrot_goal = 0;
    int16_t wrot_posi = 0;
    bool wrot_onoff = false;

    int8_t gripper_temp = 0;
    uint8_t gripper_velo = 0;
    int16_t gripper_goal = 0;
    int16_t gripper_posi = 0;
    bool gripper_onoff = false;
};

struct trial_packet
{
    uint8_t header = '\x35';
    char trial_name[20] = { 0 };
    uint32_t trial_time_start = 0;
    uint32_t trial_time_end = 0;
};

struct motion_packet
{
    uint8_t header = '\x37';
    double start_x = 0.;
    double start_y = 0.;
    double start_z = 0.;
    double start_e = 0.;

    double stop_x = 0.;
    double stop_y = 0.;
    double stop_z = 0.;
    double stop_e = 0.;

    uint32_t start_time = 0;
    uint32_t stop_time = 0;
    uint16_t joint_one_start = 0;
    uint16_t joint_one_stop = 0;
    uint16_t joint_two_start = 0;
    uint16_t joint_two_stop = 0;
    uint16_t joint_three_start = 0;
    uint16_t joint_three_stop = 0;
    uint16_t joint_four_start = 0;
    uint16_t joint_four_stop = 0;
    uint16_t joint_five_start = 0;
    uint16_t joint_five_stop = 0;
    uint16_t joint_six_start = 0;
    uint16_t joint_six_stop = 0;


};

struct network_packet
{
    uint8_t header = '\x36';
    uint32_t time_recorded = 0;
    uint32_t serial_commands_received = 0;
    uint32_t serial_gtp_received = 0;
    uint32_t serial_bad_reads = 0;
    uint8_t serial_connection_fault = 0;

    uint32_t ada_commands_received = 0;
    uint32_t ada_command_faults = 0;
    uint64_t ada_writes_received = 0;
    uint32_t ada_write_faults = 0;
    uint64_t ada_reads_received = 0;
    uint32_t ada_read_faults = 0;
    uint8_t ada_sd_fault = 0;
    uint8_t ada_connection_fault = 0;
    uint8_t ada_bme01_fault = 0;
    uint8_t ada_bme02_fault = 0;
    char ada_eng_sys_msg[30] = { 0 };

    uint32_t bbox_commands_received = 0;
    uint32_t bbox_command_faults = 0;
    uint64_t bbox_writes_received = 0;
    uint32_t bbox_write_faults = 0;
    uint64_t bbox_reads_received = 0;
    uint32_t bbox_read_faults = 0;
    uint8_t bbox_sd_fault = 0;
    uint8_t bbox_connection_fault = 0;
    char bbox_eng_sys_msg[30] = { 0 };

    uint8_t ada_commands = 0;
    uint8_t bbox_commands = 0;
    uint8_t cam_commands = 0;
    uint8_t arm_commands = 0;
    uint8_t netw_commands = 0;
};

/*
void sendData( HardwareSerial &serial, uint8_t *data, int length );
void assignEntry( char *dst, const char *src, int length, bool from_uplink = false );
TRANS_TYPE receiveData( HardwareSerial &serial, uint8_t (&buffer)[MAX_BUF], unsigned int &index );
bool bufferToReading( uint8_t (&buffer)[MAX_BUF], SENSOR_READING &reading );
bool bufferToCommand( uint8_t (&buffer)[MAX_BUF], GROUND_COMMAND &com );
bool bufferToGTP( uint8_t (&buffer)[MAX_BUF], GTP_DATA &gtp );
void resetBuffer( uint8_t (&buffer)[MAX_BUF], unsigned int &index );


void sendCommand( HardwareSerial &serial, uint8_t command );
*/

struct data_packet
{
    uint8_t header[2] = { 2, 1 }; // actuals are x01 and x21
    uint32_t time_sent_to_HASP;
    uint8_t num_data_chunks = 0U;
    uint16_t sizeof_data_chunks = 0U;
    uint8_t checksumz[CHECKSUMZ]; // all x01
    uint8_t meat[MEAT_SIZE];
    uint8_t terminator[2] = { 255, 255 }; //actuals are x03 and x0D

    void setCheckSums()
    {
        header[0] = '\x01';
        header[1] = '\x21';
        terminator[0] = '\x03';
        terminator[1] = '\x0D';
        for( int x = 0; x < CHECKSUMZ; x++ )
            checksumz[x] = '\x01';
    }

    bool verifyCheckSums()
    {
        bool result = true;

        if( header[0] == '\x01' &&
            header[1] == '\x21' &&
            terminator[0] == '\x03' &&
            terminator[1] == '\x0D' )
        {
            for( int x = 0; x < CHECKSUMZ; x++ )
            {
                if( checksumz[x] != '\x01' )
                {
                    result = false;
                    break;
                }
            }
        }
        else
        {
            result = false;
        }
        return result;
    }
    template<typename packet_type>
    bool addPacket( packet_type packet )
    {
        uint16_t space_remaining = MEAT_SIZE - sizeof_data_chunks;
        bool success = true;

        if( space_remaining >= sizeof( packet_type ) )
        {
            memcpy( &meat[sizeof_data_chunks], &packet, sizeof( packet_type ) );
            sizeof_data_chunks += sizeof( packet_type );
            num_data_chunks++;
        }
        else
        {
            success = false;
        }

        return success;
    }
};
#endif
