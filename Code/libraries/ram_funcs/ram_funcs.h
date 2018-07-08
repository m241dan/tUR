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
const uint16_t PACKET_SIZE = 507;

struct data_packet
{
    uint8_t header[2] = { 2, 1 }; // actuals are x01 and x21
    uint32_t time_sent_to_HASP;
    uint8_t num_data_chunks = 0;
    uint16_t sizeof_data_chunks;
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
};

struct image_packet
{
    uint8_t header = '\x30';
    uint8_t position;
    uint16_t photo_number;
    uint16_t sizeof_photo;
    uint8_t meat[462];
    image_packet() {}
    image_packet( uint8_t buf[] )
    {
        header = buf[0];
        position = buf[1];
        photo_number = *((uint16_t *)&buf[2]);
        sizeof_photo = *((uint16_t *)&buf[4]);
        for( int x = 0; x < 462; x++ )
            meat[x] = buf[x+6];
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
    uint8_t header = '\x33';
    uint32_t time_recorded;

    int8_t turntable_temp;
    uint8_t turntable_velo;
    int16_t turntable_goal;
    int16_t turntable_posi;
    bool turntable_onoff;

    int8_t shoulder_temp;
    uint8_t shoulder_velo;
    int16_t shoulder_goal;
    int16_t shoulder_posi;
    bool shoulder_onoff;

    int8_t elbow_temp;
    uint8_t elbow_velo;
    int16_t elbow_goal;
    int16_t elbow_posi;
    bool elbow_onoff;

    int8_t wrist_temp;
    uint8_t wrist_velo;
    int16_t wrist_goal;
    int16_t wrist_posi;
    bool wrist_onoff;

    int8_t gripper_temp;
    uint8_t gripper_velo;
    int16_t gripper_goal;
    int16_t gripper_posi;
    bool gripper_onoff;

    int16_t armposition_in_mm_X;
    int16_t armposition_in_mm_Y;
    int16_t armposition_in_mm_Z;
    int16_t armorientation_in_rads;
};

struct pathlog_packet
{
    uint8_t header = '\x35';
    double target_pos_x;
    double target_pos_y;
    double target_pos_z;
    double target_pos_angle;
    uint16_t current_pos_joint1;
    uint16_t current_pos_joint2;
    uint16_t current_pos_joint3;
    uint16_t current_pos_joint4;
    uint16_t current_pos_joint5;
    uint16_t total_trial_count;
    uint16_t trial_ID;
    uint32_t pathstep_time_start;
    uint32_t pathstep_time_end;
    uint16_t gripper_position;
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
