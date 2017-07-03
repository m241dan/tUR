#include "states.h"

virtual STATE_ID downlink_ground::run()
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
       if( blu_serial )
          sendData( *blu_serial, (byte *)&current_reading, sizeof( SENSOR_READING ) );
       sendData( ground_serial, (byte *)&current_reading, sizeof( SENSOR_READING ) );

    }
    return transition;
}

void downlink_ground::prepareReading( SENSOR_READING &reading )
{
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

    ext_temp = data.ext_temp_total / data.super_sample;
    ext_humidity = data.ext_humidity_total / data.super_sample;
}

void downlink_ground::writeSD( SENSOR_READING &reading )
{
    File goat_log;
    byte *ptr;

    if( !( goat_log = SD.open( statuss.log_name ,FILE_WRITE ) ) )
    {
        statuss.sd_status = "WRITEFAIL";
        return;
    }

    ptr = (byte *)&(reading.time[0]);

    for( int x = 0; x < sizeof( SENSOR_READING ); x++ )
    {
        goat_log.write(*ptr++);
    }

    goat_log.close();
}

virtual STATE_ID request_slave_reading::run()
{
    sendCommand( slave_serial, REQUEST_READING );
    return NONE_SPECIFIC;
}

virtual STATE_ID commander_handler::run()
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
virtual STATE_ID timer_handler::run()
{
    STATE_ID transition = SAMPLE;
    unsigned long long now_time = millis();

    /*
     * sync timers data with HASP time
     */
    if( timers.gtp_time != gtp.time )
       timers.gtp_time = gtp.time;

    /*
     * If the pump time is less than now time and the pump
     * is in automode, then toggle it. Toggle the boolean
     * letting us know if the pump is on or off. Finally,
     * set the next pump time to be now + fifteen minutes.
     * This means that next time our now_time is bigger than
     * pump time, toggle it.
     */
    if( timers.pump_time < now_time && statuss.pump_auto == true )
    {
        if( statuss.pump_on )
            pump.off();
        else
            pump.on();
        statuss.pump_on = statuss.pump_on ? false : true;
        timers.pump_time = now_time + FIFTEEN_MINUTES;
    }

    /*
     * Check if it is time to downlink, if it is
     * we use a specific transition to head to downlink
     * state.
     */
    if( timers.downlink_schedule < now_time )
    {
        timers.downlink_schedule = millis() + ( 1 * 1000 ) //downlink again a second from now
        transition = DOWNLINK_GROUND;
    }

    return transition;
}


/*
 * In this State, just take readings from all the sensors
 * and increment the super sample counter for averaging.
 */
virtual STATE_ID sample::run()
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

    return NONE_SPECIFIC
}


