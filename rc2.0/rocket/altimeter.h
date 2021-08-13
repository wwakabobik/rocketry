#ifndef __ALTIMETER_H__
#define __ALTIMETER_H__

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


void init_barometer();
float get_temperature();
float get_pressure();
float get_humidity();
float get_altitude();
String get_altimeter_data();
float get_land_altitude();

#endif
