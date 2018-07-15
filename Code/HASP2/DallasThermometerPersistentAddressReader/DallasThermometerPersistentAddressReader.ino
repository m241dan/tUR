// Testing to see if we can actually read these fuckers by address.
// EDIT: We can! Yay. Thanks to http://henrysbench.capnfatz.com/ for the tutorial. -JA

#include <OneWire.h>
#include <DallasTemperature.h>
#define THERMO_BANK_10 10
#define THERMO_BANK_11 11
#define THERMO_BANK_12 12
#define THERMO_BANK_13 13

OneWire oneWire_10(THERMO_BANK_10);
OneWire oneWire_11(THERMO_BANK_11);
OneWire oneWire_12(THERMO_BANK_12);
OneWire oneWire_13(THERMO_BANK_13);
DallasTemperature thermbank_10(&oneWire_10);
DallasTemperature thermbank_11(&oneWire_11);
DallasTemperature thermbank_12(&oneWire_12);
DallasTemperature thermbank_13(&oneWire_13);

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

void setup(void)
{
  Serial.begin(9600);
  /*
  // set the resolution to 9 bit - Valid values are 9, 10, or 11 bit.
  thermbank_10.setResolution(thermometer_08, 11);
  thermbank_10.setResolution(thermometer_10, 11);
  */
}

void loop(void)
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
  
  // Get the readings into variables.
  temperature_04 = (thermbank_10.getTempC(thermometer_04) * 100);
  temperature_08 = (thermbank_10.getTempC(thermometer_08) * 100);
  temperature_10 = (thermbank_10.getTempC(thermometer_10) * 100);
  temperature_16 = (thermbank_10.getTempC(thermometer_16) * 100);
  
  temperature_17 = (thermbank_11.getTempC(thermometer_17) * 100);
  temperature_05 = (thermbank_11.getTempC(thermometer_05) * 100);
  temperature_07 = (thermbank_11.getTempC(thermometer_07) * 100);
  temperature_09 = (thermbank_11.getTempC(thermometer_09) * 100);

  temperature_00 = (thermbank_12.getTempC(thermometer_00) * 100);
  temperature_01 = (thermbank_12.getTempC(thermometer_01) * 100);
  temperature_02 = (thermbank_12.getTempC(thermometer_02) * 100);
  temperature_03 = (thermbank_12.getTempC(thermometer_03) * 100);

  temperature_11 = (thermbank_13.getTempC(thermometer_11) * 100);
  temperature_12 = (thermbank_13.getTempC(thermometer_12) * 100);
  temperature_13 = (thermbank_13.getTempC(thermometer_13) * 100);
  temperature_06 = (thermbank_13.getTempC(thermometer_06) * 100);
  
  Serial.print("00: " + temperature_00 + "\t");
  Serial.print("01: " + temperature_01 + "\t");
  Serial.print("02: " + temperature_02 + "\t");
  Serial.print("03: " + temperature_03 + "\t");
  Serial.print("04: " + temperature_04 + "\t");
  Serial.print("05: " + temperature_05 + "\t");
  Serial.print("06: " + temperature_06 + "\t");
  Serial.print("07: " + temperature_07 + "\t");
  Serial.print("08: " + temperature_08 + "\t");
  Serial.print("09: " + temperature_09 + "\t");
  Serial.print("10: " + temperature_10 + "\t");
  Serial.print("11: " + temperature_11 + "\t");
  Serial.print("12: " + temperature_12 + "\t");
  Serial.print("13: " + temperature_13 + "\t");
  Serial.print("14: " + temperature_14 + "\t");
  Serial.print("15: " + temperature_15 + "\t");
  Serial.print("16: " + temperature_16 + "\t");
  Serial.print("17: " + temperature_17 + "\t");
  Serial.print("\tVREG 20-12: ");
  Serial.print(temperature_08);
  Serial.print("\tVREG 12-05: ");
  Serial.print(temperature_10);
  Serial.print("\tBusyboxTop: ");
  Serial.println(temperature_16);
  delay(1000);
}
