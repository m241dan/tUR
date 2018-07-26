#include <Wire.h>
#include <ram_funcs.h>
#include <ram_registers.h>
#include <hasp_arduino_sysclock.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include <ram_commands.h>
#include <SPI.h>
#include <SD.h>

#define THERMO_BANK_10 10
#define THERMO_BANK_11 11
#define THERMO_BANK_12 12
#define THERMO_BANK_13 13
#define BME_SCK        4
#define BME_MISO       2
#define BME_MOSI       3
#define BME_CS         5 //Analog2
#define BME_DS         6 //Analog3 for test

ADA_output_register output_register;
ADA_input_register  input_register;
ArduinoSysClock     sys_clock   ( output_register.time_register );
OneWire             oneWire_10(THERMO_BANK_10);
OneWire             oneWire_11(THERMO_BANK_11);
OneWire             oneWire_12(THERMO_BANK_12);
OneWire             oneWire_13(THERMO_BANK_13);
DallasTemperature   thermbank_10(&oneWire_10);
DallasTemperature   thermbank_11(&oneWire_11);
DallasTemperature   thermbank_12(&oneWire_12);
DallasTemperature   thermbank_13(&oneWire_13);
Adafruit_BME280     bme01       (BME_CS, BME_MOSI, BME_MISO, BME_SCK);
Adafruit_BME280     bme02       (BME_DS, BME_MOSI, BME_MISO, BME_SCK);
const int           SD_CARD     = 4; //"pin" for the SD card
void                (*resetFunc)(void)  = 0;
int                 write_rate  = 5000;     // 0.20 Hz or 5 seconds
unsigned long       last_write  = 0;
char log_name[15]               = { 0 };

int temperature_00, temperature_01, temperature_02, temperature_03, temperature_04 = 0;
int temperature_05, temperature_06, temperature_07, temperature_08, temperature_09 = 0;
int temperature_10, temperature_11, temperature_12, temperature_13, temperature_14 = 0;
int temperature_15, temperature_16, temperature_17 = 0;

//Living document in a more organized format exists here:
// https://docs.google.com/spreadsheets/d/1mdHR0mqF5xlRaGpWdMslQofoupi8QpfvprQK8RxCKrY/edit#gid=0

// *-------------------- Bank 10 --------------------*
    DeviceAddress thermometer_04 = {0x28, 0xFF, 0xDA, 0x15, 0x87, 0x16, 0x04, 0x2D};
    DeviceAddress thermometer_08 = {0x28, 0xFF, 0x45, 0x6C, 0x86, 0x16, 0x04, 0x65};
    DeviceAddress thermometer_10 = {0x28, 0xFF, 0x83, 0x14, 0x87, 0x16, 0x04, 0xC4};
    DeviceAddress thermometer_16 = {0x28, 0xFF, 0x02, 0x97, 0x86, 0x16, 0x04, 0x38};

// *-------------------- Bank 11 --------------------*
    DeviceAddress thermometer_17 = {0x28, 0xFF, 0xE1, 0x50, 0xB5, 0x16, 0x05, 0x77};
    DeviceAddress thermometer_05 = {0x28, 0xFF, 0xA1, 0xEB, 0x86, 0x16, 0x04, 0xFF};
    DeviceAddress thermometer_07 = {0x28, 0xFF, 0x39, 0xA3, 0x86, 0x16, 0x04, 0x8A};
    DeviceAddress thermometer_09 = {0x28, 0xFF, 0x8A, 0x74, 0x87, 0x16, 0x05, 0xF9};

// *-------------------- Bank 12 --------------------*
    DeviceAddress thermometer_00 = {0x28, 0xFF, 0xAB, 0xBD, 0xC1, 0x16, 0x04, 0xD8};
    DeviceAddress thermometer_01 = {0x28, 0xFF, 0x49, 0x30, 0xB3, 0x16, 0x05, 0x3B};
    DeviceAddress thermometer_02 = {0x28, 0xFF, 0x4E, 0x7E, 0x87, 0x16, 0x05, 0xDF};
    DeviceAddress thermometer_03 = {0x28, 0xFF, 0x9A, 0x32, 0xB3, 0x16, 0x05, 0x2C};

// *-------------------- Bank 13 --------------------*
    DeviceAddress thermometer_06 = {0x28, 0xFF, 0x6E, 0x01, 0x87, 0x16, 0x04, 0x4B};
    DeviceAddress thermometer_11 = {0x28, 0xFF, 0xB0, 0xC7, 0x85, 0x16, 0x03, 0x4B};
    DeviceAddress thermometer_12 = {0x28, 0xFF, 0xDF, 0xE3, 0x85, 0x16, 0x03, 0x4B};
    DeviceAddress thermometer_13 = {0x28, 0xFF, 0xBE, 0x44, 0x87, 0x16, 0x05, 0x2A};

// *---Homeless but listed for completion's sake---*
    /*DeviceAddress thermometer_14 = { };
    //14 BAD; FIX ME (flipped) */
    /*DeviceAddress thermometer_15 = { };
    //15 ?missing? */

/* i2c write event (onReceive) */
void writeRegisters( int num_bytes )
{
    /* If we have a complete register written in there, read it */
    if( Wire.available() == sizeof( ADA_input_register ) )
    {
        output_register.write_received = 1;
        output_register.write_fault = 0;
        output_register.command_fault = 0;
        /*
         * I don't have direct access to the buffer to do a memcpy, so I'll just use a byte pointer
         * Theoretically, this should be safe because the sizes of the register and the buffer are the "same" 
         */
        byte *input_ptr = (byte *)&input_register;
        for( int x = 0; x < sizeof( ADA_input_register ); x++ )
        {
            *input_ptr++ = Wire.read();
        }
        /*
         * Check the Data:
         *  - If it's good, reset our fault flag
         *  - If it's bad, set our fault flag
         */
        if( input_register.verifyCheckSums() )
        {
            output_register.write_fault = 0;
            if( input_register.new_sync )
                sys_clock.syncClock( input_register.sync_to, millis() );
            if( input_register.has_command )
            {
                switch( input_register.command_id )
                {
                    default:
                        output_register.command_fault = 1;
                        break;
                    case AMBIENT_WRITE_RATE[0]:
                        write_rate = (int)input_register.command_param * 1000;
                        break;
                    case AMBIENT_RESET[0]:
                        if( input_register.command_param == RESET_BYTE )
                        {
                            resetFunc();
                        }
                        else
                            output_register.command_fault = 1;
                        break;
                }
            }
        }
        else
        {
            output_register.write_fault = 1;
        }
    }
    else
    {
        output_register.write_received = 0;
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
    if( !SD.begin( SD_CARD ) )
        output_register.sd_fault = 1;
    else
        snprintf( log_name, sizeof( log_name ), "%s", getNextFile( "log", ".csv" ).c_str() );
}

void loop()
{
    // Take readings.
    thermbank_10.requestTemperaturesByAddress(thermometer_04); 
    thermbank_10.requestTemperaturesByAddress(thermometer_08); 
    thermbank_10.requestTemperaturesByAddress(thermometer_10); 
    thermbank_10.requestTemperaturesByAddress(thermometer_16); 
    
    thermbank_11.requestTemperaturesByAddress(thermometer_17); 
    thermbank_11.requestTemperaturesByAddress(thermometer_05); 
    thermbank_11.requestTemperaturesByAddress(thermometer_07); 
    thermbank_11.requestTemperaturesByAddress(thermometer_09); 
    
    thermbank_12.requestTemperaturesByAddress(thermometer_00);
    thermbank_12.requestTemperaturesByAddress(thermometer_01);
    thermbank_12.requestTemperaturesByAddress(thermometer_02);
    thermbank_12.requestTemperaturesByAddress(thermometer_03);
    
    thermbank_13.requestTemperaturesByAddress(thermometer_11);
    thermbank_13.requestTemperaturesByAddress(thermometer_12);
    thermbank_13.requestTemperaturesByAddress(thermometer_13);
    thermbank_13.requestTemperaturesByAddress(thermometer_06);

   //Thought it might be less wonky to organized it by-bank, but I can easily change it to by-thermometer. -JA
    output_register.dallas04_temp = (thermbank_10.getTempC(thermometer_04) * 100);
    output_register.dallas08_temp = (thermbank_10.getTempC(thermometer_08) * 100);
    output_register.dallas10_temp = (thermbank_10.getTempC(thermometer_10) * 100);
    output_register.dallas16_temp = (thermbank_10.getTempC(thermometer_16) * 100);

    output_register.dallas17_temp = (thermbank_11.getTempC(thermometer_17) * 100);
    output_register.dallas05_temp = (thermbank_11.getTempC(thermometer_05) * 100);
    output_register.dallas07_temp = (thermbank_11.getTempC(thermometer_07) * 100);
    output_register.dallas09_temp = (thermbank_11.getTempC(thermometer_09) * 100);
    
    output_register.dallas00_temp = (thermbank_12.getTempC(thermometer_00) * 100);
    output_register.dallas01_temp = (thermbank_12.getTempC(thermometer_01) * 100);
    output_register.dallas02_temp = (thermbank_12.getTempC(thermometer_02) * 100);
    output_register.dallas03_temp = (thermbank_12.getTempC(thermometer_03) * 100);

    output_register.dallas11_temp = (thermbank_13.getTempC(thermometer_11) * 100);
    output_register.dallas12_temp = (thermbank_13.getTempC(thermometer_12) * 100);
    output_register.dallas13_temp = (thermbank_13.getTempC(thermometer_13) * 100);
    output_register.dallas06_temp = (thermbank_13.getTempC(thermometer_06) * 100);
    
    output_register.bme01_temp = bme01.readTemperature();
    output_register.bme01_humi = bme01.readHumidity();
    output_register.bme01_pres = bme01.readPressure() / 100.0F;

    output_register.bme02_temp = bme01.readTemperature();
    output_register.bme02_humi = bme02.readHumidity();
    output_register.bme02_pres = bme02.readPressure() / 100.0F;

//Debug code for BMEs:
    Serial.print("BME01 (O.R.)= ");
    Serial.print(String(output_register.bme01_temp) + "C,\t");
    Serial.print(String(output_register.bme01_pres) + " hPa\t");
    Serial.print(String(output_register.bme01_humi) + "%,\t");

    Serial.print("BME02 (O.R.)= ");
    Serial.print(String(output_register.bme02_temp) + "C,\t");
    Serial.print(String(output_register.bme02_pres) + " hPa\t");
    Serial.println(String(output_register.bme02_humi) + "%,\t");

    Serial.print("BME01 (direct)= ");
    Serial.print(String(bme01.readTemperature()) + "C,\t");
    Serial.print(String(bme01.readHumidity()) + " hPa\t");
    Serial.print(String(bme01.readPressure() / 100.0F) + "%,\t\n");
    
    sys_clock.updateClock( millis() );

    if( !output_register.sd_fault && millis() - last_write > write_rate )
    {
        File log_file = SD.open( log_name, FILE_WRITE );
        if( !log_file )
        {
            output_register.sd_fault = 1;
        }
        else
        {
            log_file.println( output_register.serialize_csv() );
            output_register.sd_fault = 0;
            log_file.close();
        }
        last_write = millis();
    }
}
