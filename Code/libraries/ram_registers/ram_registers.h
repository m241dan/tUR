#ifndef ram_registers
#define ram_registers

#include <ram_funcs.h>

struct ADA_output_register
{
    unsigned char check_one = 0;
    unsigned long time_register;

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
    unsigned long packets_sent = 0;
    unsigned char write_fault = 0;
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
    unsigned char fresh_packet;
    unsigned char check_two = 0;
    data_packet packet;
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


struct BB_input_register
{
    unsigned char check_one = '\xBB';
    unsigned char check_two = '\xFA';
    unsigned char check_three = '\xFE';

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
