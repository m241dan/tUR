// Wire Master Writer
// Demonstrates use of the Wire library
// Writes data to an I2C/TWI slave device
// Refer to the "Wire Slave Receiver" example for use with this

#include <Wire.h>

void setup() 
{
  Wire.begin(); // join i2c bus (address optional for master)
  pinMode(2, INPUT);
}

byte x = 0;

void loop() 
{
  Wire.beginTransmission(8); // transmit to device #8
  if (digitalRead(2) == HIGH)
  {
    x++;
  }
  else
  {
    x = 0;
  }
  Wire.write("x is ");        // sends five bytes
  Wire.write(x);              // sends one byte
  Wire.endTransmission();    // stop transmitting
  delay(200);
}
