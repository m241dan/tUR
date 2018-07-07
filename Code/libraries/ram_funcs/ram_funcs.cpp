String getNextFile( String name, String append )
{
   String file_name;
   int file_iteration = 1;

   //This loop will basically find us a free file appended 1-2.1 billion, which should be plenty
   file_name = name + String( file_iteration ) + append;
   for( ; SD.exists( file_name.c_str() ); file_iteration++ )
      file_name = name + String( file_iteration ) + append;

   return file_name;
}

