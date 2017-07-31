#include "Spec.h"
#include "Arduino.h"
#define OVER_SAMPLE 16

Spec::Spec( specType s_type, int g, int r, int t, double code, double offset ) : type(s_type), vgas_pin(g), vref_pin(r), vtmp_pin(t), sensitivity_code(code), v_offset(offset)
{
   //O3 and NO2 are the same
   double TIA = 499.00;

   pinMode( vgas_pin, INPUT );
   pinMode( vref_pin, INPUT );
   pinMode( vtmp_pin, INPUT );

   vgas_reading = 0;
   vref_reading = 0;
   vtmp_reading = 0;

   //janky but it is what it is
   if( type == SPEC_SO2 )
      TIA = 100.00;
   M = sensitivity_code * (TIA * 0.000000001 ) * 1000.00;
}

Spec::Spec( specType s_type, double code, double offset ) : type(s_type), sensitivity_code(code), v_offset(offset)
{
    double TIA = 499.00

    if( type == SPEC_SO2 )
        TIA = 100.00;

    M = sensitivity_code * ( TIA * 0.000000001 ) * 1000.00;
}

String Spec::generateRawReading( char delim, bool perr, bool newline )
{
   String reading = "";
   takeReading();
   reading += String( vgas_reading ) + delim;
   reading += String( vref_reading ) + delim;
   reading += String( vtmp_reading ) + delim;

   //options
   if( perr )
       reading += String( ( vgas_reading - vref_reading ) / vref_reading );
   if( newline )
      reading += "\r\n";

   return reading;
}

String Spec::generateRawVerboseReading()
{
   String reading = "| Sensor: ";

   switch( type )
   {
      default: reading += "Unknown"; break;
      case SPEC_SO2:
         reading += "SO2";
         break;
      case SPEC_O3:
         reading += "O3";
         break;
      case SPEC_NO2:
         reading += "NO2";
         break;
   }
   reading += " | ";
   takeReading();

   reading += "VGAS: " + String( vgas_reading ) + " ";
   reading += "VREF: " + String( vref_reading ) + " ";
   reading += "VTMP: " + String( vtmp_reading ) + " ";
   reading += "%ERR: " + String( ( vgas_reading - vref_reading ) / vref_reading ) + "\r\n";

   return reading;
}

double Spec::generateReadingPPM()
{
    double reading_ppm;
    double vgas_offset;

    takeReading();

    vgas_offset = vref_reading - v_offset;

    reading_ppm = (vgas_reading - vgas_offset ) / M;
    reading_ppm = reading_ppm < 0 ? 0 : reading_ppm;
    return reading_ppm;
}

double Spec::generateReadingPPM( double differential )
{
    double reading_ppm;
    double vgas_offset;

    vgas_offset = differential - v_offset;

    reading_ppm = vgas_offset / M;
    reading_ppm = reading_ppm < 0 ? 0 : reading__ppm;

    return reading_ppm;
}
void Spec::takeReading()
{
    vgas_reading = analogRead( vgas_pin );
    delay( 10 );
    vgas_reading = 0;
    for( int c = 0; c < OVER_SAMPLE; c++ )
        vgas_reading += analogRead( vgas_pin ) * ( 5. / 1023. );
    vgas_reading /= OVER_SAMPLE;

    vref_reading = analogRead( vref_pin );
    delay( 10 );
    vref_reading = 0;
    for( int c = 0; c < OVER_SAMPLE; c++ )
        vref_reading += analogRead( vref_pin ) * ( 5. / 1023. );
    vref_reading /= OVER_SAMPLE;

    vtmp_reading = analogRead( vtmp_pin );
    delay( 10 );
    vtmp_reading = 0;
    for( int c = 0; c < OVER_SAMPLE; c++ )
        vtmp_reading += analogRead( vtmp_pin ) * ( 5. / 1023. );
    vtmp_reading /= OVER_SAMPLE;
}

double Spec::getTemperature()
{
    return ( ( 87.0 * vtmp_reading - 18.00 ) - 32.0) * ( 5.0 / 9.0 );
}
