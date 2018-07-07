#ifndef ram_registers
#define ram_registers

#include <ram_funcs.h>

struct ADA_output_register
{
    unsigned char check_one = 0;
    unsigned long time_register = 0;

    signed short bme01_temp = 0;
    signed short bme01_pres = 0;
    unsigned char bme01_humi = 0;
    unsigned char bme01_fault = 0;

    signed short bme02_temp = 0;
    signed short bme02_pres = 0;
    unsigned char bme02_humi = 0;
    unsigned char bme02_fault = 0;

    unsigned char check_two = 0;
    signed short dallas01_temp = 0;
    signed short dallas02_temp = 0;
    signed short dallas03_temp = 0;
    signed short dallas04_temp = 0;
    signed short dallas05_temp = 0;
    signed short dallas06_temp = 0;
    signed short dallas07_temp = 0;
    signed short dallas08_temp = 0;
    signed short dallas09_temp = 0;
    signed short dallas10_temp = 0;
    signed short dallas11_temp = 0;
    signed short dallas12_temp = 0;
    signed short dallas13_temp = 0;
    signed short dallas14_temp = 0;
    signed short dallas15_temp = 0;
    signed short dallas16_temp = 0;
    unsigned char check_three = 0;
    unsigned long reads_received = 0;
    unsigned long writes_received = 0; // this will roll over, and that's okay
    unsigned short commands_received = 0;
    unsigned char write_fault = 0; // write checksums failed
    unsigned char command_fault = 0;
    char english_sys_msg[30];

    void setCheckSums()
    {
        check_one = '\xDE';
        check_two = '\xAD';
        check_three = '\xAF';
    }
    bool verifyCheckSums()
    {
        bool verified = false;

        if( check_one == '\xDE' &&
            check_two == '\xAD' &&
            check_three == '\xAF' )
        {
            verified = true;
        }

        return verified;
    }
};

struct ADA_input_register
{
    unsigned char check_one = 0;
    unsigned long last_write = 0;
    unsigned char has_command = 0;
    unsigned char command_id = 0;
    unsigned char command_param = 0;
    unsigned char check_two = 0;
    unsigned char new_sync = 1;
    unsigned long sync_to = 1530844583;
    unsigned char check_three = 0;

    void setCheckSums()
    {
        check_one = '\xFE';
        check_two = '\xDA';
        check_three = '\xFF';
    }
    bool verifyCheckSums()
    {
        bool verified = false;

        if( check_one == '\xFE' &&
            check_two == '\xDA' &&
            check_three == '\xFF' )
        {
            verified = true;
        }

        return verified;
    }
};


struct BBOX_output_register
{
    unsigned char check_one = 0;
    unsigned long time_register = 0;
    unsigned char rocker_horiz = 0;
    unsigned char rocker_verti = 0;
    unsigned char toggle_horiz = 0;
    unsigned char toggle_verti = 0;
    unsigned char check_two = 0;
    unsigned char button_blu = 0;
    unsigned char potentiometer_lever = 0;
    unsigned char potentiometer_knob = 0;
    unsigned long reads_received = 0;
    unsigned long writes_received = 0; // this will roll over, and that's okay
    unsigned short commands_received = 0;
    unsigned char write_fault = 0; // write checksums failed
    unsigned char command_fault = 0;
    unsigned char check_three = 0;
    char english_sys_msg[30];

    void setCheckSums()
    {
        check_one = '\xBB';
        check_two = '\xBE';
        check_three = '\xEF';
    }

    bool verifyCheckSums()
    {
        bool verified = false;

        if( check_one == '\xBB' &&
            check_two == '\xBE' &&
            check_three == '\xEF' )
        {
            verified = true;
        }

        return verified;
    }

};

struct BBOX_input_register
{
    unsigned char check_one = 0;
    unsigned long last_write = 0;
    unsigned char check_two = 0;
    unsigned char has_command = 0;
    unsigned char command_id = 0;
    unsigned char command_param = 0;
    unsigned long sync_to = 0;
    unsigned char check_three = 0;

    void setCheckSums()
    {
        check_one = '\xBB';
        check_two = '\xFA';
        check_three = '\xFE';
    }

    bool verifyCheckSums()
    {
        bool verified = false;

        if( check_one == '\xBB' &&
            check_two == '\xFA' &&
            check_three == '\xFE' )
        {
            verified = true;
        }

        return verified;
    }

};

#endif
