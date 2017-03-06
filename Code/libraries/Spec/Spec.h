/*****************************************
 * Arduino Spec Library for Arduino Mega *
 * Written by Daniel R. Koris            *
 *****************************************/

#ifndef spec_sensor_h
#define spec_sensor_h

#include "Arduino.h"

typedef enum : byte
{
   SPEC_SO2, SPEC_O3, SPEC_NO2
} specType;

class Spec
{
   public:
      Spec( specType s_type, int g, int r, int t );
      ~Spec();

      String generateReading();

   private:
      //Vars
      specType type;
      int vgas_pin;
      int vref_pin;
      int vtmp_pin;
      int pow_one;
      int pow_two;

};

#endif
