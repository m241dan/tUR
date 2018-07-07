#include <Wire.h>
#include <ram_funcs.h>
#include <ram_registers.h>
#include <hasp_arduino_sysclock.h>
#include <DallasTemperature.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include <ram_commands.h>

#define THERMO_CH1 9 //WHT
#define THERMO_CH2 8 //BLU
#define THERMO_CH3 7 //WHT
#define THERMO_CH4 6 //BLU
#define BME_SCK 24
#define BME_MISO 22
#define BME_MOSI 23 
#define BME_CS 0
#define BME_DS 1

ADA_output_register output_register;
ADA_input_register  input_register;
ArduinoSysClock     sys_clock   ( output_register.time_register );
OneWire             oneWire_ch1 ( THERMO_CH1 );
OneWire             oneWire_ch2 ( THERMO_CH2 );
OneWire             oneWire_ch3 ( THERMO_CH3 );
OneWire             oneWire_ch4 ( THERMO_CH4 );
DallasTemperature   sensors_ch1 ( &oneWire_ch1 );
DallasTemperature   sensors_ch2 ( &oneWire_ch2 );
DallasTemperature   sensors_ch3 ( &oneWire_ch3 );
DallasTemperature   sensors_ch4 ( &oneWire_ch4 );
Adafruit_BME280     bme01       (BME_CS, BME_MOSI, BME_MISO, BME_SCK);
Adafruit_BME280     bme02       (BME_DS, BME_MOSI, BME_MISO, BME_SCK);

/* i2c write event (onReceive) */
void writeRegisters( int num_bytes )
{
    /* If we have a complete register written in there, read it */
    if( Wire.available() == sizeof( ADA_input_register ) )
    {
        /*
         * I don't have direct access to the buffer to do a memcpy, so I'll just use a byte pointer
         * Theoretically, this should be safe because the sizes of the register and the buffer are the "same" 
         */
        byte *input_ptr = (byte *)&input_register;
        for( int x = 0; x < sizeof( ADA_input_register ); x++ )
        {
            *input_ptr++ = Wire.read();
        }
        output_register.writes_received++;
        /*
         * Check the Data:
         *  - If its good, reset our fault flag
         *  - If its bad, set out fault flag
         */
        if( input_register.verifyCheckSums() )
        {
            output_register.write_fault = 0;
            if( input_register.new_sync )
                sys_clock.syncClock( input_register.sync_to, millis() );    
        }
        else
        {
            output_register.write_fault = 1;
        }
    }
}

/* i2c read event (onRequest) */
void readRegisters()
{
    output_register.setCheckSums();
    Wire.write( (byte *)&output_register, sizeof( ADA_output_register ) );
}

void setup()
{
    Wire.begin( I2CADDRESS_ADA );
    Wire.onRequest( readRegisters );
    Wire.onReceive( writeRegisters );
    if( !bme01.begin() )
        output_register.bme01_fault = 1;
    if( !bme02.begin() )
        output_register.bme02_fault = 1;
        
    sys_clock.syncClock( 1234470131, millis() );
}

void loop()
{
    sensors_ch1.requestTemperatures();
    sensors_ch2.requestTemperatures();
    sensors_ch3.requestTemperatures();
    sensors_ch4.requestTemperatures();

    output_register.dallas01_temp = sensors_ch1.getTempCByIndex(0);
    output_register.dallas02_temp = sensors_ch1.getTempCByIndex(1);
    output_register.dallas03_temp = sensors_ch1.getTempCByIndex(2);
    output_register.dallas04_temp = sensors_ch1.getTempCByIndex(3);
    
    output_register.dallas05_temp = sensors_ch1.getTempCByIndex(0);
    output_register.dallas06_temp = sensors_ch1.getTempCByIndex(1);
    output_register.dallas07_temp = sensors_ch1.getTempCByIndex(2);
    output_register.dallas08_temp = sensors_ch1.getTempCByIndex(3);
    
    output_register.dallas09_temp = sensors_ch1.getTempCByIndex(0);
    output_register.dallas10_temp = sensors_ch1.getTempCByIndex(1);
    output_register.dallas11_temp = sensors_ch1.getTempCByIndex(2);
    output_register.dallas12_temp = sensors_ch1.getTempCByIndex(3);
    
    output_register.dallas13_temp = sensors_ch1.getTempCByIndex(0);
    output_register.dallas14_temp = sensors_ch1.getTempCByIndex(1);
    output_register.dallas15_temp = sensors_ch1.getTempCByIndex(2);
    output_register.dallas16_temp = sensors_ch1.getTempCByIndex(3);

    output_register.bme01_temp = bme01.readTemperature();
    output_register.bme01_humi = bme01.readHumidity();
    output_register.bme01_pres = bme01.readPressure() / 100.0F;

    output_register.bme02_temp = bme01.readTemperature();
    output_register.bme02_humi = bme02.readHumidity();
    output_register.bme02_pres = bme02.readPressure() / 100.0F;
    
    sys_clock.updateClock( millis() );
}
