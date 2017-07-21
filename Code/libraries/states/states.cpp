#include "states.h"

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

STATE_ID receive_slave::run()
{
    TRANS_TYPE transmission;

    if( ( transmission = receiveData( refs.slave_serial, refs.buffers.slave, refs.buffers.slave_index ) ) != TRANS_INCOMPLETE )
        bufferToReading( refs.buffers.slave, refs.readings.slave );

    return NONE_SPECIFIC;
}

STATE_ID downlink_ground::run()
{
    STATE_ID transition = NONE_SPECIFIC;
    bool downlink = false;
    SENSOR_READING *current_reading = &refs.readings.master;


    switch( refs.statuss.which_bank )
    {
        case 1:
            prepareReading( *current_reading );
            downlink = true;
            break;
        case 2:
            current_reading = &refs.readings.slave;
            if( current_reading->header[0] == 0 )
            {
                if( refs.statuss.slave_wait_sanity++ > 10 )
                {
                    refs.statuss.which_bank = 1;
                    refs.statuss.slave_wait_sanity = 0;
                    transition = REQUEST_SLAVE_READING;
                }
                downlink = false;
            }
            else
                downlink = true;
            break;
    }

    if( downlink )
    {
       writeSD( *current_reading );
       sendData( refs.blu_serial, (byte *)current_reading, sizeof( SENSOR_READING ) );
       sendData( refs.ground_serial, (byte *)current_reading, sizeof( SENSOR_READING ) );

       memset( current_reading, 0, sizeof( SENSOR_READING ) );

       refs.statuss.which_bank = refs.statuss.which_bank  == 1 ? 2 : 1;
       if( refs.statuss.which_bank == 2 )
          transition = REQUEST_SLAVE_READING;
    }
    return transition;
}

void downlink_ground::prepareReading( SENSOR_READING &reading )
{
    double synced_now_time;
    double so2_ppm;
    double no2_ppm;
    double o3_ppm;
    double temp;
    double humidity;
    double pressure;
    double ext_temp;
    double ext_humidity;

    so2_ppm = refs.sample_set.so2_total / refs.sample_set.super_sample;
    no2_ppm = refs.sample_set.no2_total / refs.sample_set.super_sample;
    o3_ppm  = refs.sample_set.o3_total / refs.sample_set.super_sample;

    temp = refs.sample_set.temp_total / refs.sample_set.super_sample;
    humidity = refs.sample_set.humidity_total / refs.sample_set.super_sample;
    pressure = refs.sample_set.pressure_total / refs.sample_set.super_sample;
    refs.statuss.goat_pressure = pressure;

    ext_temp = refs.sample_set.ext_temp_total / refs.sample_set.super_sample;
    ext_humidity = refs.sample_set.ext_humidity_total / refs.sample_set.super_sample;

    synced_now_time = refs.readings.gtp.utc_time + ( ( millis() - refs.timers.gtp_received_at ) / 1000.00F );

    reading.header[0] = '\x01';
    reading.header[1] = '\x21';
    reading.terminator[0] = '\r';
    reading.terminator[1] = '\n';

    assignEntry( reading.time, String( synced_now_time ).c_str(), sizeof( reading.time ) );
    assignEntry( reading.bank, "1", sizeof( reading.bank ) );
    assignEntry( reading.so2_reading, String( so2_ppm ).c_str(), sizeof( reading.so2_reading ) );
    assignEntry( reading.no2_reading, String( no2_ppm ).c_str(), sizeof( reading.no2_reading ) );
    assignEntry( reading.o3_reading, String( o3_ppm ).c_str(), sizeof( reading.o3_reading ) );
    assignEntry( reading.temp_reading, String( temp ).c_str(), sizeof( reading.temp_reading ) );
    assignEntry( reading.extt_reading, String( ext_temp ).c_str(), sizeof( reading.extt_reading ) );
    assignEntry( reading.pressure_reading, String( pressure ).c_str(), sizeof( reading.pressure_reading ) );
    assignEntry( reading.humidity_reading, String( ext_humidity ).c_str(), sizeof( reading.humidity_reading ) );

    /*
     * Determine the Status of the Bump from the Status Table
     */
    assignEntry( reading.pump_status, pump_status_string[refs.statuss.pump_auto], sizeof( reading.pump_status ) );
    assignEntry( reading.bme_status, refs.statuss.bme_status.c_str(), sizeof( reading.pump_status ) );
    assignEntry( reading.am2315_status, refs.statuss.am2315_status.c_str(), sizeof( reading.am2315_status ) );
    assignEntry( reading.sd_status, refs.statuss.sd_status.c_str(), sizeof( reading.sd_status ) );
    assignEntry( reading.reading_status, "ACT AUTO", sizeof( reading.reading_status ) );
}

void downlink_ground::writeSD( SENSOR_READING &reading )
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

    ptr = (byte *)&(reading.time[0]);

    for( int x = 0; x < sizeof( SENSOR_READING ); x++ )
    {
        goat_log.write(*ptr++);
    }

    goat_log.close();
}

STATE_ID request_slave_reading::run()
{
    sendCommand( refs.slave_serial, REQUEST_READING );
    return NONE_SPECIFIC;
}

STATE_ID command_handler::run()
{

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
    if( refs.timers.gtp_time != refs.readings.gtp.utc_time )
       refs.timers.gtp_time = refs.readings.gtp.utc_time;

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
            if( refs.statuss.goat_pressure > 100.00 )
               refs.statuss.pump_auto = PUMP_OFF_PRESSURE;
            else if( refs.timers.pump_timer < now_time )
            {
                refs.statuss.pump_auto = PUMP_OFF_AUTO;
                refs.timers.pump_timer = now_time + FIFTEEN_MINUTES;
            }
            break;
        case PUMP_OFF_AUTO:
            if( refs.statuss.goat_pressure > 100.00 )
               refs.statuss.pump_auto = PUMP_OFF_PRESSURE;
            else if( refs.timers.pump_timer < now_time )
            {
                refs.statuss.pump_auto = PUMP_ON_AUTO;
                refs.timers.pump_timer = now_time + FIFTEEN_MINUTES;
            }
            break;
        case PUMP_OFF_PRESSURE:
            if( refs.statuss.goat_pressure < 100.00 )
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
        refs.timers.downlink_schedule = millis() + ( 1 * 1000 ); //downlink again a second from now
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
    refs.sample_set.so2_total += refs.sensors.so2.generateReadingPPM();
    refs.sample_set.no2_total += refs.sensors.no2.generateReadingPPM();
    refs.sample_set.o3_total += refs.sensors.o3.generateReadingPPM();
    refs.sample_set.temp_total += refs.sensors.bme.readTemperature();
    refs.sample_set.humidity_total += refs.sensors.bme.readHumidity();
    refs.sample_set.pressure_total += refs.sensors.bme.readPressure() / 100.0F;
    refs.sample_set.ext_temp_total += refs.sensors.dongle.readTemperature();
    refs.sample_set.ext_humidity_total += refs.sensors.dongle.readHumidity();
    refs.sample_set.super_sample++;

    return NONE_SPECIFIC;
}


