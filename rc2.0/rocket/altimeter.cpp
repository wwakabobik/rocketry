#include "altimeter.h"


// Sensor globals
float normal_pressure = 0;
float land_altitude = 0;
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();


void init_barometer()
{
    if (!bme.begin())
    {
        Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
        while (1) delay(10);
    }

    sensors_event_t pressure_event;
    bme_pressure->getEvent(&pressure_event);
    normal_pressure = pressure_event.pressure;

    land_altitude = get_altitude();

    #ifdef DEBUG
    Serial.println("Init BME280 OK");
    bme_temp->printSensorDetails();
    bme_pressure->printSensorDetails();
    bme_humidity->printSensorDetails();
    #endif
}

/* BME functions */

float get_temperature()
{
    sensors_event_t temp_event, pressure_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    return temp_event.temperature;
}


float get_pressure()
{
    sensors_event_t pressure_event;
    bme_pressure->getEvent(&pressure_event);
    return (pressure_event.pressure/1.3333);
}


float get_humidity()
{
    float humidity;
    sensors_event_t humidity_event;
    bme_humidity->getEvent(&humidity_event);
    humidity = humidity_event.relative_humidity;
    return humidity;
}


float get_altitude()
{
    float altitude;
    sensors_event_t altitude_event;
    altitude = bme.readAltitude(normal_pressure);;
}


String get_altimeter_data()
{
    String ret_str = "";
    ret_str = String(get_temperature()) + "," + String(get_humidity());
    ret_str += "," + String(get_pressure()) + "," + String(get_altitude());
    return ret_str;
}

float get_land_altitude()
{
    return land_altitude;
}
