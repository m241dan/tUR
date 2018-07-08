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

struct data_packet
{
    uint8_t datapacket_header[2] = {'\x1', '\x21'};
    uint8_t datapacket_i2cID;
    uint32_t datapacket_time_sent_to_HASP;
    uint32_t datapacket_time_sent_to_mngr;
    uint8_t datapacket_num_data_chunks;
    uint8_t datapacket_sizeof_data_chunks[2];
    uint8_t datapacket_checksumz[28];
    uint8_t datapacket_meat[468];
    uint8_t datapacket_terminator[2] = {'\x03', '\x0D'};
};

struct image_packet
{
    uint8_t imagepacket_header = '\x30';
    uint8_t imagepacket_position;
    uint16_t imagepacket_photo_number;
    uint16_t imagepacket_sizeof_photo;
    uint8_t imagepacket_meat[462];
    image_packet() {}
    image_packet( uint8_t buf[] )
    {
        imagepacket_header = buf[0];
        imagepacket_position = buf[1];
        imagepacket_photo_number = *((uint16_t *)&buf[2]);
        imagepacket_sizeof_photo = *((uint16_t *)&buf[4]);
        for( int x = 0; x < 462; x++ )
            imagepacket_meat[x] = buf[x+6];
    }
};

struct ambient_packet
{
    uint8_t header = '\x31';
    uint32_t time_recorded;

    int16_t bme01_temp;
    int16_t bme01_pres;
    uint8_t bme01_humi;

    int16_t bme02_temp;
    int16_t bme02_pres;
    uint8_t bme02_humi;

    int16_t dallas01_temp;
    int16_t dallas02_temp;
    int16_t dallas03_temp;
    int16_t dallas04_temp;
    int16_t dallas05_temp;
    int16_t dallas06_temp;
    int16_t dallas07_temp;
    int16_t dallas08_temp;
    int16_t dallas09_temp;
    int16_t dallas10_temp;
    int16_t dallas11_temp;
    int16_t dallas12_temp;
    int16_t dallas13_temp;
    int16_t dallas14_temp;
    int16_t dallas15_temp;
    int16_t dallas16_temp;
};

struct bbox_packet
{
    uint8_t header = '\x32';
    uint32_t time_recorded;

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
    uint8_t arm_header = '\x33';
    uint32_t arm_time_recorded;

    int8_t arm_turntable_temp;
    uint8_t arm_turntable_velo;
    int16_t arm_turntable_goal;
    int16_t arm_turntable_posi;
    bool arm_turntable_onoff;

    int8_t arm_shoulder_temp;
    uint8_t arm_shoulder_velo;
    int16_t arm_shoulder_goal;
    int16_t arm_shoulder_posi;
    bool arm_shoulder_onoff;

    int8_t arm_elbow_temp;
    uint8_t arm_elbow_velo;
    int16_t arm_elbow_goal;
    int16_t arm_elbow_posi;
    bool arm_elbow_onoff;

    int8_t arm_wrist_temp;
    uint8_t arm_wrist_velo;
    int16_t arm_wrist_goal;
    int16_t arm_wrist_posi;
    bool arm_wrist_onoff;

    int8_t arm_gripper_temp;
    uint8_t arm_gripper_velo;
    int16_t arm_gripper_goal;
    int16_t arm_gripper_posi;
    bool arm_gripper_onoff;

    int16_t arm_armposition_in_mm_X;
    int16_t arm_armposition_in_mm_Y;
    int16_t arm_armposition_in_mm_Z;
    int16_t arm_armorientation_in_rads;
};

struct pathlog_packet
{
    uint8_t pathlog_header = '\x35';
    double pathlog_target_pos_x;
    double pathlog_target_pos_y;
    double pathlog_target_pos_z;
    double pathlog_target_pos_angle;
    uint16_t pathlog_current_pos_joint1;
    uint16_t pathlog_current_pos_joint2;
    uint16_t pathlog_current_pos_joint3;
    uint16_t pathlog_current_pos_joint4;
    uint16_t pathlog_current_pos_joint5;
    uint16_t pathlog_total_trial_count;
    uint16_t pathlog_trial_ID;
    uint32_t pathlog_pathstep_time_start;
    uint32_t pathlog_pathstep_time_end;
    uint16_t pathlog_gripper_position;
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
#endif
