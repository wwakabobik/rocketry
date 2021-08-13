#include "gps.h"

// GPS globals
double lat, lon, alt, kmph;  // GPS data are saved here: Latitude, Longitude, Altitude, Speed in km/h
int sats;  // GPS satellite count
char s[32];  // used to sprintf for Serial output
const long GPS_DELAY = 120000;
HardwareSerial GPSSerial(1);
gps gps_wrapper;


void gps::init()
{
    GPSSerial.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
    GPSSerial.setTimeout(2);
}

void gps::encode()
{
    int data;
    int previousMillis = millis();

    while((previousMillis + 1000) > millis())
    {
        while (GPSSerial.available() )
        {
            char data = GPSSerial.read();
            tGps.encode(data);
        }
    }
}

void gps::getLatLon(double* lat, double* lon, double *alt, double *kmph, int *sats)
{
    #ifdef ROCKET_DEBUG
    sprintf(t, "Lat: %f", tGps.location.lat());
    Serial.println(t);

    sprintf(t, "Lng: %f", tGps.location.lng());
    Serial.println(t);

    sprintf(t, "Alt: %f meters", tGps.altitude.meters());
    Serial.println(t);

    sprintf(t, "Speed: %f km/h", tGps.speed.kmph());
    Serial.println(t);

    sprintf(t, "Sats: %d", tGps.satellites.value());
    Serial.println(t);
    #endif

    *lat = tGps.location.lat();
    *lon = tGps.location.lng();
    *alt = tGps.altitude.meters();
    *kmph = tGps.speed.kmph();
    *sats = tGps.satellites.value();
}

bool gps::checkGpsFix()
{
    encode();
    if (tGps.location.isValid() &&
      tGps.location.age() < 2000 &&
      tGps.hdop.isValid() &&
      tGps.hdop.value() <= 300 &&
      tGps.hdop.age() < 2000 &&
      tGps.altitude.isValid() &&
      tGps.altitude.age() < 2000 )
    {
        #ifdef ROCKET_DEBUG
        Serial.println("Valid gps Fix.");
        #endif
        return true;
    }
    else
    {
        #ifdef ROCKET_DEBUG
        Serial.println("No gps Fix.");
        #endif
        return false;
    }
}


void init_GPS()
{
    gps_wrapper.init();
    delay(GPS_DELAY);
    #ifdef ROCKET_DEBUG
    Serial.println(print_GPS_data(True));
    Serial.println("GPS started");
    #endif
}


// GPS functions
String get_GPS_data()
{
    if (gps_wrapper.checkGpsFix())
    {
        // Prepare upstream data transmission at the next possible time.
        gps_wrapper.getLatLon(&lat, &lon, &alt, &kmph, &sats);
    }
    else
    {
        #ifdef ROCKET_DEBUG
        Serial.println("GPS is not ready");
        #endif
    }
}

String print_GPS_data(bool verbose)
{
    String return_string = "";
    if (verbose)
    {
        return_string += "GPS data:\nLat: ";
    }
    return_string += String(lat);
    if (verbose)
    {
        return_string += "\tLon: ";
    }
    return_string += String(lon);
    if (verbose)
    {
        return_string += "\tAlt: ";
    }
    return_string += String(alt);
    if (verbose)
    {
        return_string += " m\tSpeed: ";
    }
    return_string += String(kmph);
    if (verbose)
    {
        return_string += " km/h\tSatellites: ";
    }
    return_string += String(sats);
    return return_string;
}
