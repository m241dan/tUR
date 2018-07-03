extern "C" {
    #include <wiringPi.h>
    #include <wiringPiI2C.h>
}
int main( int argc, char *argv[] )
{
    int fd = wiringPiI2CRead( 1 );

    return 0;
}
