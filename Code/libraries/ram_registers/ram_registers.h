#ifndef ram_registers
#define ram_registers

#include <ram_funcs.h>

struct ADA_output_register
{
    unsigned char check_one = 0;
    unsigned long time_register = 0;

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
    unsigned char check_two = 0;
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
    unsigned long last_write;
    unsigned char has_command;
    unsigned char command_id;
    unsigned char command_param;
    unsigned char check_two = 0;
    unsigned char new_sync = 0;
    unsigned long sync_to = 1530844583;;
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
    unsigned long time_register;
    unsigned char bbox_rocker_horiz;
    unsigned char bbox_rocker_verti;
    unsigned char bbox_toggle_horiz;
    unsigned char bbox_toggle_verti;
    unsigned char check_two = 0;
    unsigned char bbox_button_blu;
    unsigned char bbox_potentiometer_lever;
    unsigned char bbox_potentiomer_knob;
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
    unsigned long last_write;
    unsigned char check_two = 0;
    unsigned char has_command;
    unsigned char command_id;
    unsigned char command_param;
    unsigned long sync_to;
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
