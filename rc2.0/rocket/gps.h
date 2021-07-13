#ifndef __GPS_H__
#define __GPS_H__

#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define GPS_TX 12
#define GPS_RX 15

// GPS globals
double lat, lon, alt, kmph;  // GPS data are saved here: Latitude, Longitude, Altitude, Speed in km/h
int sats;  // GPS satellite count
char s[32];  // used to sprintf for Serial output
const long GPS_DELAY = 120000;


class gps
{
    public:
        void init();
        bool checkGpsFix();
        void getLatLon(double* lat, double* lon, double *alt, double *kmph, int *sats);
        void encode();

    private:
        char t[32]; // used to sprintf for Serial output
        TinyGPSPlus tGps;
};


void init_GPS();
void get_GPS_data();
String print_GPS_data(bool verbose);


#endif