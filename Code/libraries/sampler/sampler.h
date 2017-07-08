#ifndef sampler_h
#define sampler_h



class Sampler
{
    public:
        Sampler();
        ~Sampler();

    private:
        Spec so2;
        double so2_ppm;
        int so2_count;

        Spec no2;
        double no2_ppm;
        int no2_count;

        Spec o3;
        double o3_ppm;
        int o3_count;

        Adafruit_BME280 bme;
        double pressure;
        double temperature;
        double humidity;
        int bme_count;

        Adafruit_AM2315 am2315;
        double ext_temp;
        double ext_humidity;
        int ext_count;
};

#endif
