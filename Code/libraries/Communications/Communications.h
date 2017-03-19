/***************************************************
 * Arduino Communications Library for Arduino Mega *
 * Specifically for the GOAT payload               *
 * Written by Daniel R. Koris                      *
 ***************************************************/

#ifndef communications_h
#define communications_h

#include "Arduino.h"

typedef enum : byte
{
   HASP_PORT, MASTER_PORT, SLAVE_PORT
} port_types;

class Communication
{
   public:
      Communication( int baud_zero, int baud_one, int baud_two, int baud_three );
      ~Communication();
   private:
      Serial ports[4];
};

#endif
