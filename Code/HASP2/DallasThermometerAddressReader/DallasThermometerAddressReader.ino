//Tester to pull IDs of Dallas thermometers 

#include <OneWire.h>
#define THERMOPIN 13

OneWire  ds(THERMOPIN);  // This is the pin to connect to

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
  
  while(ds.search(addr)) {
    Serial.println("The address is:\t");
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
  delay(20000);
  // do nothing
}
