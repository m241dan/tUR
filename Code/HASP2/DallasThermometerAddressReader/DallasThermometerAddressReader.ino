//Tester to pull IDs of Dallas thermometers 
//Make sure that you only connect one at a time when pulling addresses, obvs.
//I don't know how it will handle multiples and don't care to experiment. -JA
//I have it set to only read every 10 seconds; seems to support hot-swapping just fine,
//but it's easier to copy from serial monitor with a low report rate. Change const below.

#include <OneWire.h>
#define THERMOPIN 10
#define READING_FREQUENCY_SECONDS 10

OneWire  ds(THERMOPIN);  // This is the pin to connect to

int milliseconds = READING_FREQUENCY_SECONDS * 1000;

void setup(void) 
{
  Serial.begin(9600);
  getDeviceAddress();
}

void getDeviceAddress(void) 
{
  byte i;
  byte addr[8];
  
  Serial.println("Getting the address...");
  /* initiate a search for the OneWire object we created and read its value into
  addr array we declared above*/
  
  while(ds.search(addr)) 
  {
    //read each byte in the address array
    for( i = 0; i < 8; i++) {
      Serial.print("0x");
      if (addr[i] < 16) {
        Serial.print('0');
      }
      // print each byte in the address array in hex format
      Serial.print(addr[i], HEX);
      if (i < 7) {
        Serial.print(", ");
      }
    }
    Serial.println(" ");
    // a check to make sure that what we read is correct.
    if ( OneWire::crc8( addr, 7) != addr[7]) {
        Serial.print("CRC is not valid!\n");
        return;
    }
  }
  ds.reset_search();
  return;
}

void loop(void) {
  getDeviceAddress();
  delay(milliseconds);
  // do nothing
}
