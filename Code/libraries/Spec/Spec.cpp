#include "Spec.h"

Spec::Spec( specType s_type, int g, int r, int t, double code ) : type(s_type), vgas_pin(g), vref_pin(r), vtmp_pin(t), sensitivity_code(code)
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
   M = 1.00 / ( ( sensitivity_code * (TIA * TIA_E ) ) * 1000 );
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

int Spec::generateReadingPPM()
{
    double reading_ppm;

    takeReading();

    reading_ppm = M * (vgas_reading - vref_reading );

    return reading_ppm;
}

void Spec::takeReading()
{
    vgas_reading = analogRead( vgas_pin ) * ( 5.0 / 1024.0 );
    vref_reading = analogRead( vref_pin ) * ( 5.0 / 1024.0 );
    vtmp_reading = analogRead( vtmp_pin ) * ( 5.0 / 1024.0 );
}
