#include "states.h"

void( *resetFunc) (void) = 0;


STATE_ID receive_ground::run()
{
    STATE_ID transition = NONE_SPECIFIC;
    TRANS_TYPE transmission;

    if( ( transmission = receiveData( refs.ground_serial, refs.buffers.ground, refs.buffers.ground_index ) ) != TRANS_INCOMPLETE )
    {
        switch( transmission )
        {
           case TRANS_COMMAND:
              bufferToCommand( refs.buffers.ground, refs.ground_command_handle );
              //this is essentially a checksum test
              if( refs.ground_command_handle.command[0] == refs.ground_command_handle.command[1] )
                  transition = COMMAND_HANDLER;
              break;
           case TRANS_GTP:
              bufferToGTP( refs.buffers.ground, refs.readings.gtp );
              refs.timers.gtp_received_at = millis();
              break;
        }
    }
    return transition;
}

STATE_ID downlink_ground::run()
{
    SENSOR_READING current_reading;


    if( !refs.statuss.write_sd )
        refs.statuss.sd_status = "SD NO WRITE";
    switch( refs.statuss.which_bank )
    {
        case 1:
            current_reading = prepareReading( 1 );
            break;
        case 2:
            current_reading = prepareReading( 2 );
            break;
    }

    if( refs.statuss.write_sd )
        writeSD( &current_reading );

    if( refs.statuss.downlink_on )
    {
        sendData( refs.ground_serial, (byte *)&current_reading, sizeof( SENSOR_READING ) );
        refs.statuss.which_bank = refs.statuss.which_bank  == 1 ? 2 : 1;
    }
    return NONE_SPECIFIC;
}

SENSOR_READING downlink_ground::prepareReading( byte bank )
{
    SENSOR_READING reading;
    double synced_addition;
    char addition_buffer[15];
    char synced_now_time[15];
    double so2_ppm;
    double no2_ppm;
    double o3_ppm;
    double temp;
    double humidity;
    double pressure;
    double ext_temp;
    double ext_humidity;

    DATA_SET *sample_set = bank == 1 ? &refs.babi_set : &refs.goat_set;

    so2_ppm = sample_set->so2_total / sample_set->super_sample;
    no2_ppm = sample_set->no2_total / sample_set->super_sample;
    o3_ppm  = sample_set->o3_total / sample_set->super_sample;

    temp = sample_set->temp_total / sample_set->super_sample;
    humidity = sample_set->humidity_total / sample_set->super_sample;
    pressure = sample_set->pressure_total / sample_set->super_sample;
    refs.statuss.babi_pressure = pressure;

    ext_temp = sample_set->ext_temp_total / sample_set->super_sample;
    ext_humidity = sample_set->ext_humidity_total / sample_set->super_sample;

    /* begin fancy hack magic to get a current time in seconds from january 1 1970 */
    synced_addition = ( millis() - refs.timers.gtp_received_at ) / 1000.00F;
    strcpy( addition_buffer, String( synced_addition ).c_str() );
    int synced_cross_over_index = strlen( refs.readings.gtp.utc_time ) - strlen( addition_buffer );
    if( synced_cross_over_index < 0 )
        synced_cross_over_index = 0;

    memset( &synced_now_time[0], 0, 15 );
    for( int x = 0; x < 15; x++ )
    {
        if( x < synced_cross_over_index )
            synced_now_time[x] = refs.readings.gtp.utc_time[x];
        else
            synced_now_time[x] = addition_buffer[x-synced_cross_over_index];
    }

    reading.header[0] = '\x01';
    reading.header[1] = '\x21';
    reading.terminator[0] = '\r';
    reading.terminator[1] = '\n';

    assignEntry( reading.time, synced_now_time, sizeof( reading.time ) );
    assignEntry( reading.bank, bank == 1 ? "1" : "2", sizeof( reading.bank ) );
    assignEntry( reading.so2_reading, String( so2_ppm ).c_str(), sizeof( reading.so2_reading ) );
    assignEntry( reading.no2_reading, String( no2_ppm ).c_str(), sizeof( reading.no2_reading ) );
    assignEntry( reading.o3_reading, String( o3_ppm ).c_str(), sizeof( reading.o3_reading ) );
    assignEntry( reading.temp_reading, String( temp ).c_str(), sizeof( reading.temp_reading ) );
    assignEntry( reading.extt_reading, String( ext_temp ).c_str(), sizeof( reading.extt_reading ) );
    assignEntry( reading.pressure_reading, String( pressure ).c_str(), sizeof( reading.pressure_reading ) );
    assignEntry( reading.humidity_reading, String( humidity ).c_str(), sizeof( reading.humidity_reading ) );
    assignEntry( reading.ext_humidity_reading, String( ext_humidity ).c_str(), sizeof( reading.ext_humidity_reading ) );

    /*
     * Determine the Status of the Bump from the Status Table
     */
    assignEntry( reading.pump_status, pump_status_string[(int)refs.statuss.pump_auto], sizeof( reading.pump_status ) );
    assignEntry( reading.bme_status, refs.statuss.which_bank == 1 ? refs.statuss.babi_bme_status.c_str() : refs.statuss.goat_bme_status.c_str(), sizeof( reading.bme_status ) );
    assignEntry( reading.am2315_status, refs.statuss.am2315_status.c_str(), sizeof( reading.am2315_status ) );
    assignEntry( reading.sd_status, refs.statuss.sd_status.c_str(), sizeof( reading.sd_status ) );
    if( refs.statuss.reading_auto )
        assignEntry( reading.reading_status, "READ ON", sizeof( reading.reading_status ) );
    else
        assignEntry( reading.reading_status, "READ OFF", sizeof( reading.reading_status ) );

    memset( sample_set, 0, sizeof( DATA_SET ) );
    return reading;
}

void downlink_ground::writeSD( SENSOR_READING *reading )
{
    File goat_log;
    byte *ptr;

    if( !( goat_log = SD.open( refs.statuss.log_name ,FILE_WRITE ) ) )
    {
        if( refs.statuss.sd_status == "SD INIT G" )
            refs.statuss.sd_status = "WRITEFAIL";
        return;
    }
    else
       refs.statuss.sd_status == "SD INIT G";

    ptr = (byte *)(reading->time[0]);

    for( int x = 0; x < sizeof( SENSOR_READING ); x++ )
    {
        goat_log.write(*ptr++);
    }

    goat_log.close();
}

STATE_ID command_handler::run()
{
    GROUND_COMMAND &com = refs.ground_command_handle;

    switch( com.command[0] )
    {
        default:
        case ACKNOWLEDGE:
            break;
        case ARD_RESET:
            resetFunc();
            break;
        case DOWNLINK_OFF:
            refs.statuss.downlink_on = false;
            break;
        case DOWNLINK_ON:
            refs.statuss.downlink_on = true;
            break;
        case STOP_SENSORS:
            refs.statuss.reading_auto = false;
            break;
        case START_SENSORS:
            refs.statuss.reading_auto = true;
            break;
        case PUMP_ON:
            if( refs.statuss.pump_auto == PUMP_ON_MANUAL )
                refs.statuss.pump_auto = PUMP_ON_AUTO;
            else
                refs.statuss.pump_auto = PUMP_ON_MANUAL;
            break;
        case PUMP_OFF:
            if( refs.statuss.pump_auto == PUMP_OFF_MANUAL )
                refs.statuss.pump_auto = PUMP_OFF_AUTO;
            else
                refs.statuss.pump_auto = PUMP_OFF_MANUAL;
            break;
        case DISABLE_SD:
            refs.statuss.write_sd = false;
            break;
        case ENABLE_SD:
            refs.statuss.write_sd = true;
            break;
        case REINIT_SD:
            if( !SD.begin( SD_PIN ) )
            {
                refs.statuss.sd_status = "SD INIT F";
            }
            else
            {
                refs.statuss.sd_status = "SD INIT G";
                refs.statuss.log_name = getNextFile( LOG_NAME );
            }
            break;
    }

    return NONE_SPECIFIC;
}

/*
 * The Timer State has two major functions: Controlling
 * the Pump and Transitioning to a Downlink State.
 * If also monitors and syncs time with data from HASP.
 * It also transitions into either sampling or downlinking
 * depending on the time.
 */
STATE_ID timer_handler::run()
{
    STATE_ID transition = SAMPLE;
    unsigned long long now_time = millis();
    /*
     * sync timers data with HASP time
     */
    if( strcmp( refs.timers.gtp_time, refs.readings.gtp.utc_time ) )
       strcpy( refs.timers.gtp_time, refs.readings.gtp.utc_time );

    /*
     * If the pump time is less than now time and the pump
     * is in automode, then toggle it. Toggle the boolean
     * letting us know if the pump is on or off. Finally,
     * set the next pump time to be now + fifteen minutes.
     * This means that next time our now_time is bigger than
     * pump time, toggle it.
     */

    //begin ghetto hack state machine, ya baby
    //transition
    switch( refs.statuss.pump_auto )
    {
        case PUMP_ON_AUTO:
//            if( refs.statuss.babi_pressure > 100.00 )
 //             refs.statuss.pump_auto = PUMP_OFF_PRESSURE;
   //         else
            if( refs.timers.pump_timer < now_time )
            {
                refs.statuss.pump_auto = PUMP_OFF_AUTO;
                refs.timers.pump_timer = now_time + FIFTEEN_MINUTES;
            }
            break;
        case PUMP_OFF_AUTO:
//            if( refs.statuss.babi_pressure > 100.00 )
  //             refs.statuss.pump_auto = PUMP_OFF_PRESSURE;
    //        else
            if( refs.timers.pump_timer < now_time )
            {
                refs.statuss.pump_auto = PUMP_ON_AUTO;
                refs.timers.pump_timer = now_time + FIFTEEN_MINUTES;
            }
            break;
        case PUMP_OFF_PRESSURE:
//            if( refs.statuss.babi_pressure < 100.00 )
                refs.statuss.pump_auto = PUMP_ON_AUTO;
            break;
        case PUMP_ON_MANUAL:
        case PUMP_OFF_MANUAL:
            break;
    }
    //action
    switch( refs.statuss.pump_auto )
    {
        case PUMP_ON_AUTO:
        case PUMP_ON_MANUAL:
            refs.pump.on();
            break;
        case PUMP_OFF_AUTO:
        case PUMP_OFF_PRESSURE:
        case PUMP_OFF_MANUAL:
            refs.pump.off();
            break;
    }
    //end super awesome ghetto hack state machine

    /*
     * Check if it is time to downlink, if it is
     * we use a specific transition to head to downlink
     * state.
     */
    if( refs.timers.downlink_schedule < now_time )
    {
        refs.timers.downlink_schedule = millis() + ( 4 * 1000 ); //downlink again a second from now
        transition = DOWNLINK_GROUND;
    }

    return transition;
}


/*
 * In this State, just take readings from all the sensors
 * and increment the super sample counter for averaging.
 */
STATE_ID sample::run()
{
    if( refs.statuss.reading_auto )
    {
        refs.sensors.dongle.readSensor();
        refs.babi_set.so2_total += ( refs.sensors.babi_ads_so2_no2.readADC_Differential_0_1() * ADS_GAIN_CONVERSION_FACTOR ) / 1000.00;
        refs.babi_set.no2_total += ( refs.sensors.babi_ads_so2_no2.readADC_Differential_2_3() * ADS_GAIN_CONVERSION_FACTOR ) / 1000.00;
        refs.babi_set.o3_total += ( refs.sensors.babi_ads_o3.readADC_Differential_0_1() * ADS_GAIN_CONVERSION_FACTOR ) / 1000.00;
        refs.babi_set.temp_total += refs.sensors.babi_bme.readTemperature();
        refs.babi_set.humidity_total += refs.sensors.babi_bme.readHumidity();
        refs.babi_set.pressure_total += refs.sensors.babi_bme.readPressure() / 100.0F;
        refs.babi_set.ext_temp_total += refs.sensors.dongle.getTemperature_C();
        refs.babi_set.ext_humidity_total += refs.sensors.dongle.getHumidity();
        refs.babi_set.super_sample++;

        refs.goat_set.so2_total += ( refs.sensors.goat_ads_so2_no2.readADC_Differential_0_1() * ADS_GAIN_CONVERSION_FACTOR ) / 1000.00;
        refs.goat_set.no2_total += ( refs.sensors.goat_ads_so2_no2.readADC_Differential_2_3() * ADS_GAIN_CONVERSION_FACTOR ) / 1000.00;
        refs.goat_set.o3_total += ( refs.sensors.goat_ads_o3.readADC_Differential_0_1() * ADS_GAIN_CONVERSION_FACTOR ) / 1000.00;
        refs.goat_set.temp_total += refs.sensors.goat_bme.readTemperature();
        refs.goat_set.humidity_total += refs.sensors.goat_bme.readHumidity();
        refs.goat_set.pressure_total += refs.sensors.goat_bme.readPressure() / 100.0F;
        refs.goat_set.ext_temp_total += refs.sensors.dongle.getTemperature_C();
        refs.goat_set.ext_humidity_total += refs.sensors.dongle.getHumidity();
        refs.goat_set.super_sample++;
    }

    return NONE_SPECIFIC;
}


