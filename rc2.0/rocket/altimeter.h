#ifndef __ALTIMETER_H__
#define __ALTIMETER_H__


// Sensor globals
float normal_pressure = 0;
float land_altitude = 0;
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
Adafruit_Sensor *bme_altitude = bmp.readAltitude(normal_pressure);


void init_barometer();
float get_temperature();
float get_pressure();
float get_humidity();
float get_altitude();
String get_altimeter_data();

#endif