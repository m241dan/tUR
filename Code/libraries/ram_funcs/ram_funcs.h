#ifndef ram_funcs_h
#define ram_funcs_h

#include "time.h"
typedef unsigned char byte;
const unsigned char DEADZONE_WIDTH = 1; //By how many units (range 0-100) does the potentiometer ignore noise. 2 may be a touch high; consider nudging down to 1 later. -JA

// I2C ADDRESSING
// Arduino pin 4 (the data, or SDA, pin) is WHT wire
// Arduino pin 5 (the clock, or SCL, pin) is YEL wire
const unsigned char I2CADDRESS_BBOX = 0x02;
const unsigned char I2CADDRESS_ADA  = 0x04;

struct data_packet
{
    unsigned char datapacket_header[2] = {'\x1', '\x21'};
    unsigned char datapacket_i2cID;
    time_t datapacket_time_sent_to_HASP;
    time_t datapacket_time_sent_to_mngr;
    unsigned char datapacket_num_data_chunks;
    unsigned char datapacket_sizeof_data_chunks[2];
    unsigned char datapacket_checksumz[28];
    unsigned char datapacket_meat[468];
    unsigned char datapacket_terminator[2] = {'\x3', '\xD'};
};

struct image_packet
{
    unsigned char imagepacket_header = '\x30';
    unsigned char imagepacket_position;
    unsigned short imagepacket_photo_number;
    unsigned short imagepacket_sizeof_photo;
    unsigned char imagepacket_meat[462];
    image_packet() {}
    image_packet( byte buf[] )
    {
        imagepacket_header = buf[0];
        imagepacket_position = buf[1];
        imagepacket_photo_number = *((unsigned short *)&buf[2]);
        imagepacket_sizeof_photo = *((unsigned short *)&buf[4]);
        for( int x = 0; x < 462; x++ )
            imagepacket_meat[x] = buf[x+6];
    }
};

struct ambient_packet
{
    unsigned char ambpacket_header = '\x31';
    time_t ambpacket_time_recorded;

    signed short ambpacket_bme01_temp;
    signed short ambpacket_bme01_pres;
    unsigned char ambpacket_bme01_humi;

    signed short ambpacket_bme02_temp;
    signed short ambpacket_bme02_pres;
    unsigned char ambpacket_bme02_humi;

    signed short ambpacket_bme03_temp;
    signed short ambpacket_bme03_pres;
    unsigned char ambpacket_bme03_humi;

    signed short ambpacket_bme04_temp;
    signed short ambpacket_bme04_pres;
    unsigned char ambpacket_bme04_humi;

    signed short ambpacket_dallas01_temp;
    signed short ambpacket_dallas02_temp;
    signed short ambpacket_dallas03_temp;
    signed short ambpacket_dallas04_temp;
    signed short ambpacket_dallas05_temp;
    signed short ambpacket_dallas06_temp;
    signed short ambpacket_dallas07_temp;
    signed short ambpacket_dallas08_temp;
    signed short ambpacket_dallas09_temp;
    signed short ambpacket_dallas10_temp;
    signed short ambpacket_dallas11_temp;
    signed short ambpacket_dallas12_temp;
    signed short ambpacket_dallas13_temp;
    signed short ambpacket_dallas14_temp;
    signed short ambpacket_dallas15_temp;
    signed short ambpacket_dallas16_temp;
};

struct arm_packet
{
    unsigned char arm_header = '\x33';
    time_t arm_time_recorded;

    signed char arm_turntable_temp;
    unsigned char arm_turntable_velo;
    signed short arm_turntable_goal;
    signed short arm_turntable_posi;
    bool arm_turntable_onoff;

    signed char arm_shoulder_temp;
    unsigned char arm_shoulder_velo;
    signed short arm_shoulder_goal;
    signed short arm_shoulder_posi;
    bool arm_shoulder_onoff;

    signed char arm_elbow_temp;
    unsigned char arm_elbow_velo;
    signed short arm_elbow_goal;
    signed short arm_elbow_posi;
    bool arm_elbow_onoff;

    signed char arm_wrist_temp;
    unsigned char arm_wrist_velo;
    signed short arm_wrist_goal;
    signed short arm_wrist_posi;
    bool arm_wrist_onoff;

    signed char arm_gripper_temp;
    unsigned char arm_gripper_velo;
    signed short arm_gripper_goal;
    signed short arm_gripper_posi;
    bool arm_gripper_onoff;

    signed short arm_armposition_in_mm_X;
    signed short arm_armposition_in_mm_Y;
    signed short arm_armposition_in_mm_Z;
    signed short arm_armorientation_in_rads;
};

struct bbox_packet
{
    unsigned char bbox_header = '\x32';
    time_t bbox_time_recorded;

    byte bbox_rocker_horiz = 0;
    byte bbox_rocker_verti = 0;
    byte bbox_toggle_horiz = 0;
    byte bbox_toggle_verti = 0;
    byte bbox_button_blu = 0;
    int bbox_button_blu_press_recorded = 0;
    byte bbox_flap = 0;
    byte bbox_potentiometer_lever = 0;
    byte bbox_potentiometer_knob = 0;

    //Overloaded operator to support struct-to-struct comparison. Needs to be replaced with memcmp.
    bool operator==( const bbox_packet &rhs )
    {
        bool same = true;
        if( bbox_rocker_horiz != rhs.bbox_rocker_horiz )
        {
            same = false;
        }
        if( bbox_rocker_verti != rhs.bbox_rocker_verti )
        {
            same = false;
        }
        if( bbox_toggle_horiz != rhs.bbox_toggle_horiz )
        {
            same = false;
        }
        if( bbox_toggle_verti != rhs.bbox_toggle_verti )
        {
            same = false;
        }
        if( bbox_button_blu_press_recorded != rhs.bbox_button_blu_press_recorded )
        {
            same = false;
        }
        if( bbox_flap != rhs.bbox_flap )
        {
            same = false;
        }
        //Put in a dead zone so that the pots don't give false-positive changes.
        if((bbox_potentiometer_lever > (rhs.bbox_potentiometer_lever + DEADZONE_WIDTH)) ||
           (bbox_potentiometer_lever < (rhs.bbox_potentiometer_lever - DEADZONE_WIDTH)))
        {
            same = false;
        }
        //Put in a dead zone so that the pots don't give false-positive changes.
        if((bbox_potentiometer_knob > (rhs.bbox_potentiometer_knob + DEADZONE_WIDTH)) ||
           (bbox_potentiometer_knob < (rhs.bbox_potentiometer_knob - DEADZONE_WIDTH)))
        {
            same = false;
        }
        return same;
    };
};

struct pathlog_packet
{
    unsigned char pathlog_header = '\x35';
    double pathlog_target_pos_x;
    double pathlog_target_pos_y;
    double pathlog_target_pos_z;
    double pathlog_target_pos_angle;
    unsigned short pathlog_current_pos_joint1;
    unsigned short pathlog_current_pos_joint2;
    unsigned short pathlog_current_pos_joint3;
    unsigned short pathlog_current_pos_joint4;
    unsigned short pathlog_current_pos_joint5;
    unsigned short pathlog_total_trial_count;
    unsigned short pathlog_trial_ID;
    time_t pathlog_pathstep_time_start;
    time_t pathlog_pathstep_time_end;
    unsigned short pathlog_gripper_position;
};

/*
void sendData( HardwareSerial &serial, byte *data, int length );
void assignEntry( char *dst, const char *src, int length, bool from_uplink = false );
TRANS_TYPE receiveData( HardwareSerial &serial, byte (&buffer)[MAX_BUF], unsigned int &index );
bool bufferToReading( byte (&buffer)[MAX_BUF], SENSOR_READING &reading );
bool bufferToCommand( byte (&buffer)[MAX_BUF], GROUND_COMMAND &com );
bool bufferToGTP( byte (&buffer)[MAX_BUF], GTP_DATA &gtp );
void resetBuffer( byte (&buffer)[MAX_BUF], unsigned int &index );
String getNextFile( String name, String append );

void sendCommand( HardwareSerial &serial, unsigned char command );
*/
#endif
