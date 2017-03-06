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

   switch( type )
   {
      default: reading += "Unknown "; break;
      case SPEC_SO2:
         reading += "SO2 ";
         break;
      case SPEC_O3:
         reading += "O3 ";
         break;
      case SPEC_NO2:
         reading += "NO2 ";
         break;
   }

   reading += "| Vgas: ";
   reading += analogRead( vgas_pin );
   reading += " ";

   reading += "| Vref: ";
   reading += analogRead( vref_pin );
   reading += " ";

   reading += "| Vtmp: ";
   reading += analogRead( vtmp_pin );
   reading += " ";

   return reading;
}
