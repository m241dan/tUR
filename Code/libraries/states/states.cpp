#include "states.h"

STATE_ID receive_ground::run()
{
    STATE_ID transition = NONE_SPECIFIC;
    TRANS_TYPE transmission;

    if( ( transmission = receiveData( ground_serial, buffer, index ) ) != TRANS_INCOMPLETE )
    {
        switch( transmission )
        {
           case TRANS_COMMAND:
              bufferToCommand( buffer, command_handle );
              transition = COMMAND_HANDLER;
              break;
           case TRANS_GTP:
              bufferToGTP( buffer, gtp );
              break;
        }
    }

    return transition;
}

STATE_ID receive_slave::run()
{
    TRANS_TYPE transmission;

    if( ( transmission = receiveData( slave_serial, buffer, index ) ) != TRANS_INCOMPLETE )
        bufferToReading( buffer, reading );

    return NONE_SPECIFIC;
}

STATE_ID downlink_ground::run()
{
    STATE_ID transition = NONE_SPECIFIC;
    bool downlink = false;
    SENSOR_READING &current_reading = readings.master;


    switch( statuss.which_bank )
    {
        case 1:
            prepareReading( current_reading );
            downlink = true;
            break;
        case 2:
            current_reading = readings.slave;
            if( current_reading.header[0] == 0 )
            {
                if( statuss.slave_wait_sanity++ > 10 )
                {
                    statuss.which_bank = 1;
                    statuss.slave_wait_sanity = 0;
                    transition = REQUEST_READING;
                }
                downlink = false;
            }
            else
                downlink = true;
            break;
    }

    if( downlink )
    {
       writeSD( current_reading );
       sendData( blu_serial, (byte *)&current_reading, sizeof( SENSOR_READING ) );
       sendData( ground_serial, (byte *)&current_reading, sizeof( SENSOR_READING ) );

       memset( &current_reading, 0, sizeof( SENSOR_READING ) );

       statuss.which_bank = statuss.which_bank  == 1 ? 2 : 1;
       if( statuss.which_bank == 2 )
          transition = REQUEST_READING;
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

    so2_ppm = data.so2_total / data.super_sample;
    no2_ppm = data.no2_total / data.super_sample;
    o3_ppm  = data.o3_total / data.super_sample;

    temp = data.temp_total / data.super_sample;
    humidity = data.humidity_total / data.super_sample;
    pressure = data.pressure_total / data.super_sample;
    statuss.goat_pressure = pressure;

    ext_temp = data.ext_temp_total / data.super_sample;
    ext_humidity = data.ext_humidity_total / data.super_sample;

    synced_now_time = readings.gtp.utc_time + ( ( millis() - timers.gtp_received_at ) / 100.00F );

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
    String pump_message = "P: ";
    if( statuss.pump_on )
       pump_message += "ON ";
    else
       pump_message += "OFF ";

    if( statuss.pump_auto )
       pump_message += "AUTO";
    else
       pump_message += "MANU";

    assignEntry( reading.pump_status, pump_message.c_str(), sizeof( reading.pump_status ) );
    assignEntry( reading.bme_status, statuss.bme_status.c_str(), sizeof( reading.pump_status ) );
    assignEntry( reading.am2315_status, statuss.am2315_status.c_str(), sizeof( reading.am2315_status ) );
    assignEntry( reading.sd_status, statuss.sd_status.c_str(), sizeof( reading.sd_status ) );
    assignEntry( reading.reading_status, "ACT AUTO", sizeof( reading.reading_status ) );
}

void downlink_ground::writeSD( SENSOR_READING &reading )
{
    File goat_log;
    byte *ptr;

    if( !( goat_log = SD.open( statuss.log_name ,FILE_WRITE ) ) )
    {
        if( statuss.sd_status == "SD INIT G" )
            statuss.sd_status = "WRITEFAIL";
        return;
    }
    else
       statuss.sd_status == "SD INIT G";

    ptr = (byte *)&(reading.time[0]);

    for( int x = 0; x < sizeof( SENSOR_READING ); x++ )
    {
        goat_log.write(*ptr++);
    }

    goat_log.close();
}

STATE_ID request_slave_reading::run()
{
    sendCommand( slave_serial, REQUEST_READING );
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
    if( timers.gtp_time != gtp.utc_time )
       timers.gtp_time = gtp.utc_time;

    /*
     * If the pump time is less than now time and the pump
     * is in automode, then toggle it. Toggle the boolean
     * letting us know if the pump is on or off. Finally,
     * set the next pump time to be now + fifteen minutes.
     * This means that next time our now_time is bigger than
     * pump time, toggle it.
     */
    if( timers.pump_timer < now_time && statuss.pump_auto == true )
    {
        if( statuss.goat_pressure < 100.00 )
        {
            if( statuss.pump_on )
                pump.off();
            else
                pump.on();
            statuss.pump_on = statuss.pump_on ? false : true;
            timers.pump_timer = now_time + FIFTEEN_MINUTES;
        }
        else
        {
           pump.off();
           statuss.pump_on = false;
            timers.pump_timer = now_time + ( 1000 * 60 ); //ie, check back in a minute
        }
    }

    /*
     * Check if it is time to downlink, if it is
     * we use a specific transition to head to downlink
     * state.
     */
    if( timers.downlink_schedule < now_time )
    {
        timers.downlink_schedule = millis() + ( 1 * 1000 ); //downlink again a second from now
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
    data.so2_total += sensors.so2.generateReadingPPM();
    data.no2_total += sensors.no2.generateReadingPPM();
    data.o3_total += sensors.o3.generateReadingPPM();
    data.temp_total += sensors.bme.readTemperature();
    data.humidity_total += sensors.bme.readHumidity();
    data.pressure_total += sensors.bme.readPressure() / 100.0F;
    data.ext_temp_total += sensors.dongle.readTemperature();
    data.ext_humidity_total += sensors.dongle.readHumidity();
    data.super_sample++;

    return NONE_SPECIFIC;
}



