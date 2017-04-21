#include "Spec.h"

Spec::Spec( specType s_type, int g, int r, int t ) : type(s_type), vgas_pin(g), vref_pin(r), vtmp_pin(t)
{
   pinMode( vgas_pin, INPUT );
   pinMode( vref_pin, INPUT );
   pinMode( vtmp_pin, INPUT );
}

Spec::~Spec()
{

}

String Spec::generateReading()
{
   String reading = "| Type: ";
   double v[3];

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

   v[0] = analogRead( vgas_pin ) * ( 5.0 / 1024.0 );
   v[1] = analogRead( vref_pin );
   v[2] = analogRead( vtmp_pin );

   for( unsigned int x = 0; x < 3; x++ )
      reading += " " + String( v[x] );

   return reading;
}
