/*****************************************
 * Arduino Spec Library for Arduino Mega *
 * Written by Daniel R. Koris            *
 *****************************************/

#ifndef spec_sensor_h
#define spec_sensor_h

#include "Arduino.h"

#define SO2_TIA = 100
#define O3_TIA  = 499
#define NO2_TIA = 499
const double TIA_E = 10^(-9);

typedef enum : byte
{
   SPEC_SO2, SPEC_O3, SPEC_NO2
} specType;

class Spec
{
   public:
      Spec( specType s_type, int g, int r, int t, double code );
      Spec();

      String generateRawReading( char delim, bool perr, bool newline );
      String generateRawVerboseReading();
      int generateReadingPPM();
   private:
      //Functions
      double rawReadingToPPM( int vgas, int vref, int vtmp );
      void takeReading();
      //Vars
      specType type;
      int vgas_pin;
      int vref_pin;
      int vtmp_pin;
      double vgas_reading;
      double vref_reading;
      double vtmp_reading;
      double sensitivity_code;
      double M;

};

#endif
