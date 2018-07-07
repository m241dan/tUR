//Testing to see if we can actually read these fuckers by address.

#include <OneWire.h>
#include <DallasTemperature.h>
//OH MY GOD FIX THIS
//OH MY GOD FIX THIS
//OH MY GOD FIX THIS
//OH MY GOD FIX THIS//OH MY GOD FIX THIS
#define THERMO_BANK_10 13
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

// Thermometer 08: VReg 12->5V
DeviceAddress thermometer_08 = {0x28, 0xFF, 0x45, 0x6C, 0x86, 0x16, 0x04, 0x65};
DeviceAddress thermometer_10 = {0x28, 0xFF, 0x83, 0x14, 0x87, 0x16, 0x04, 0xC4};

// TODO REPLACE THIS BUT JUST TESTING FOR NOW; JA
float temp_08 = 0;
float temp_10 = 0;

void setup(void)
{
  // start serial port
  Serial.begin(9600);
  
  // set the resolution to 9 bit - Valid values are 9, 10, or 11 bit.
  thermbank_10.setResolution(thermometer_08, 9);
  thermbank_10.setResolution(thermometer_10, 9);
  // confirm that we set that resolution by asking the DS18B20 to repeat it back
  Serial.print("Sensor Resolution 08: ");
  Serial.println(thermbank_10.getResolution(thermometer_08), DEC); 
  Serial.print("Sensor Resolution 08: ");
  Serial.println(thermbank_10.getResolution(thermometer_10), DEC); 
  Serial.println();
}

void loop(void)
{ 
  // Tell the Sensor to Measure and Remember the Temperature it Measured
  thermbank_10.requestTemperaturesByAddress(thermometer_08); // Send the command to get temperatures
  thermbank_10.requestTemperaturesByAddress(thermometer_10); // Send the command to get temperatures
  // Get the temperature that you told the sensor to measure
  temp_08 = thermbank_10.getTempC(thermometer_08);
  temp_10 = thermbank_10.getTempC(thermometer_10);
  
  Serial.print("\tTherm 08: ");
  Serial.print(temp_08);
  Serial.print("\tTherm 10: ");
  Serial.println(temp_10);
  delay(1000);
}
