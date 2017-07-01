#ifndef hasp_types_h
#define hasp_types_h

#define ACKNOWLEDGE '\x06'
#define ARD_RESET '\x20'
#define DOWNLINK_OFF '\x21'
#define DOWNLINK_ON '\x22'
#define STOP_SENSORS '\x23'
#define START_SENSORS '\x24'
#define PUMP_ON '\x25'
#define PUMP_OFF '\x26'
#define DISABLE_SD '\x29'
#define ENABLE_SD '\x2A'
#define REINIT_SD '\x2B'
#define BANK_ONE '\x2E'
#define BANK_TWO '\x2F'
#define REQUEST_READING '\x30'

typedef enum
{
   TRANS_INCOMPLETE = 0, TRANS_COMMAND, TRANS_DATA, TRANS_GTP
} TRANS_TYPE;

typedef struct ground_command
{
   unsigned char header[2] = "\x1\x2";
   unsigned char checksum = 0;
   unsigned char command[2] = { 0, 0 };
   unsigned char terminator[3] = "\x3\xD\xA";
} GROUND_COMMAND;

typedef struct gps_time_position
{
   unsigned char header[2] = "\x1\x30";
   unsigned char data[119] = "1234470131.649,$GPGGA,202212.00,3024.7205,N,09110.7264,W,1,06,1.69,00061,M,-025,M,,*51,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,";
   unsigned char terminator[3] = "\x3\xD\xA";
} GTP_DATA;

typedef struct sensor_readings
{
   unsigned char header[2] = "\x1\x21";
   unsigned char time[15];
   unsigned char bank[2];
   unsigned char so2_reading[10];
   unsigned char no2_reading[10];
   unsigned char o3_reading[10];
   unsigned char temp_reading[5];
   unsigned char extt_reading[5];
   unsigned char pressure_reading[10];
   unsigned char humidity_reading[5];
   unsigned char ext_humidity_reading[5];
   unsigned char pump_status[10];
   unsigned char peltier_status[22];
   unsigned char sd_status[10];
   unsigned char reading_status[10];
   unsigned char terminator[2] = "\r\n";
} SENSOR_READING;

typedef struct sensor_table
{
    sensor_table() : so2( SPEC_SO2, A0, A1, A2, 43.54 ),
                     no2( SPEC_NO2, A3, A4, A5, 43.53 ),
                     o3( SPEC_O3, A6, A7, A8, 43.54 ),
                     bme( BME_PIN ) {}
    Spec so2;
    Spec no2;
    Spec o3;
    Adafruit_BME280 bme;
    Adafruit_AM2315 dongle;
} SENSOR_TABLE;

typedef struct readings_table
{
    readings_table() { memset( &master, 0, sizeof( master ) );
                       memset( &slave, 0, sizeof( slave ) );
                       memset( &gtp, 0, sizeof( gtp ) ); }
    SENSOR_READING master;
    SENSOR_READING slave;
    GTP_DATA gtp;
} READINGS_TABLE;

typedef struct status_table
{
    status_table() : log_name(""), pump_on(false), pump_auto(true),
                     reading_status(""), reading_auto(true), sd_status(""),
                     bme_status(""), am2315_status(""), which_bank(1) {}
    String log_name;
    bool pump_on;
    bool pump_auto;
    String reading_status;
    bool reading_auto;
    String sd_status;
    String bme_status;
    String am2315_status;
    byte which_bank;
} STATUS_TABLE;

typedef struct receive_buffers
{
    receive_buffers() : ground_index(0), slave_index(0) {
                        memset( &ground[0], 0, MAX_BUF );
                        memset( &slave[0], 0, MAX_BUF ); }
    byte ground[MAX_BUF];
    unsigned int ground_index;
    byte slave[MAX_BUF];
    unsigned int slave_index;
} RECEIVE_BUFFERS;

typedef struct timer_table
{
    unsigned long long downlink_schedule = 0;
    unsigned long long gtp_time = 0;
    unsigned long long gtp_received_at = 0;
    unsigned long long pump_timer = 0;
    int slave_wait_sanity = 0;
} TIMER_TABLE;

typedef struct data_set
{
    double so2_total = 0;
    double no2_total = 0;
    double o3_total = 0;
    double temp_total = 0;
    double humidity_total = 0;
    double pressure_total = 0;
    double ext_temp_total = 0;
    double ext_humidity_total = 0;
    unsigned int super_sample = 0;
} DATA_SET;

#endif


