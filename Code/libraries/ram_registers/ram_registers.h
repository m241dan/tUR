#ifndef ram_registers
#define ram_registers

#include <ram_funcs.h>
#include <cstring>

struct NetworkHealth
{
    uint32_t system_time = 0;
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

    uint8_t ada_commands;
    uint8_t bbox_commands;
    uint8_t cam_commands;
    uint8_t arm_commands;
    uint8_t netw_commands;

    char *serialize_csv()
    {
        static char buf[512];
        memset( &buf[0], 0, sizeof( buf ) );

        snprintf( buf, sizeof( buf ), "%d,%d,%d,%d,%d,%d,%d,%lu,%d,%lu,%d,%d,%d,%d,%d,%s,%d,%d,%lu,%d,%lu,%d,%d%d,%s,%d,%d,%d,%d,%d",
                  system_time,serial_commands_received, serial_gtp_received, serial_bad_reads, serial_connection_fault,
                  ada_commands_received, ada_command_faults, ada_writes_received, ada_write_faults, ada_reads_received,
                  ada_read_faults, ada_sd_fault, ada_connection_fault, ada_bme01_fault, ada_bme02_fault, ada_eng_sys_msg,
                  bbox_commands_received, bbox_command_faults, bbox_writes_received, bbox_write_faults, bbox_reads_received,
                  bbox_read_faults, bbox_sd_fault, bbox_connection_fault, bbox_eng_sys_msg, ada_commands, bbox_commands,
                  cam_commands, arm_commands, netw_commands
        );
        return buf;
    }
};

struct ADA_output_register
{
    uint8_t check_one = 0;
    uint32_t time_register = 0;

    int16_t bme01_temp = 0;
    int16_t bme01_pres = 0;
    uint8_t bme01_humi = 0;
    uint8_t bme01_fault = 0;

    int16_t bme02_temp = 0;
    int16_t bme02_pres = 0;
    uint8_t bme02_humi = 0;
    uint8_t bme02_fault = 0;

    uint8_t check_two = 0;
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
    uint8_t check_three = 0;
    uint8_t write_received = 0;
    uint8_t command_received = 0;
    uint8_t write_fault = 0; // write checksums failed
    uint8_t command_fault = 0;
    uint8_t sd_fault = 0;
    char english_sys_msg[30];

    char *serialize_csv()
    {
        static char buf[512];
        memset( &buf[0], 0, sizeof( buf ) );

        snprintf( buf, sizeof( buf ), "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
            time_register, bme01_pres, bme01_temp, bme01_humi, bme02_pres, bme02_temp, bme02_humi,
            dallas01_temp, dallas02_temp, dallas03_temp, dallas04_temp, dallas05_temp, dallas06_temp,
            dallas07_temp, dallas08_temp, dallas09_temp, dallas10_temp, dallas11_temp, dallas12_temp,
            dallas13_temp, dallas14_temp, dallas15_temp, dallas16_temp );
        return buf;

    }

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
    uint8_t check_one = 0;
    uint32_t last_write = 0;
    uint8_t has_command = 0;
    uint8_t command_id = 0;
    uint8_t command_param = 0;
    uint8_t check_two = 0;
    uint8_t new_sync = 1;
    uint32_t sync_to = 1530844583;
    uint8_t check_three = 0;

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
    uint8_t check_one = 0;
    uint32_t time_register = 0;
    uint8_t rocker_horiz = 0;
    uint8_t rocker_verti = 0;
    uint8_t toggle_horiz = 0;
    uint8_t toggle_verti = 0;
    uint8_t check_two = 0;
    uint8_t button_blu = 0;
    uint8_t potentiometer_lever = 0;
    uint8_t potentiometer_knob = 0;
    uint8_t write_received = 0; // this will roll over, and that's okay
    uint16_t commands_received = 0;
    uint8_t write_fault = 0; // write checksums failed
    uint8_t command_fault = 0;
    uint8_t sd_fault = 0;
    uint8_t check_three = 0;
    char english_sys_msg[30];

    char *serialize_csv()
    {
        static char buf[512];
        memset( &buf[0], 0, sizeof( buf ) );

        snprintf( buf, sizeof( buf ), "%d,%d,%d,%d,%d,%d,%d,%d",
                  time_register, rocker_horiz, rocker_verti,
                  toggle_horiz, toggle_verti, button_blu,
                  potentiometer_lever, potentiometer_knob );
        return buf;

    }

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
    uint8_t check_one = 0;
    uint32_t last_write = 0;
    uint8_t check_two = 0;
    uint8_t has_command = 0;
    uint8_t command_id = 0;
    uint8_t command_param = 0;
    uint32_t sync_to = 0;
    uint8_t check_three = 0;

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
