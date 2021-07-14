/* ***************************************************************************
 * This sketch contains logic of rc-controlled flight computer.              *
 *                                                                           *
 * Sketch uses TTGO Meshtastic T-Beam board as base.                         *
 * All communication via LoRA module RF433, with connected BMP180,  MPU9050, *
 * relay(s) for chute deploy.                                                *
 *                                                                           *
 * Logic:                                                                    *
 *    1) Activate GPS, LoRa, BMP180, MPU9050, pins                           *
 *    2) If something wrong while init, start beep continuously              *
 *    3) Wait for GPS is connected                                           *
 *    4) Wait for control command, response accordingly                      *
 *    5) If command is "armed", start dump sensor data tp SPIFFS             *
 *    6) If jumper is out or altitude raise more 10 meters, start telemetry  *
 *    7) If altitude starts to falling more 5 meters after maximum altitude  *
 *       value, deploy chute                                                 *
 *    8) If chute timeout is reached but p. 7 is false, deploy chute         *
 *    9) Start beeping                                                       *
 *    10) If landed or flight timeout reached, stop telemetry, send GPS data *
 *                                                                           *
 * Sketch written by Iliya Vereshchagin 2021.                                *
 *****************************************************************************/

#define DEBUG

#include <SPI.h>              
#include <LoRa.h>
#include <WiFi.h>
#include "FS.h"
#include "SPIFFS.h"

#include "gps.h"
#include "beeper.h"
#include "lora_module.h"
#include "gyro.h"
#include "altimeter.h"
#include "config.h"


// State
const int STATE_DEFAULT = 0;
const int STATE_ROCKET_CONTROL = 128;
const int STATE_ROCKET_ARMED = 140;
const int STATE_ROCKET_IN_FLIGHT = 160;
const int STATE_ROCKET_IN_FLIGHT_LANDING = 180;
const int STATE_ROCKET_LANDED = 220;
const int STATE_ROCKET_ERROR = 240;


// Ignitor const
const int IGNITOR_PIN = 14;
const int IGNITOR_CONTROL_PIN = 33;


// SPIFFS globals



void setup()
{
    #ifdef DEBUG
    Serial.begin(9600);
    #endif
    init_beeper();
    init_ignitor();
    init_LoRa();
    init_barometer();
    init_gyro();
    init_SPIFFS();
    init_GPS();
    wait_for_LoRa_command();
    wait_for_start();
}


void loop()
{
    // Seems we need to do several actions here
    // 1. get data from sensors
    // 2. Check for apogee / landing / timeouts
    // 3. Write to SPIFFS
    // 4. Send telemetry
}


// Init functions


void init_communication()
{
    //Turn off WiFi and Bluetooth
    WiFi.mode(WIFI_OFF);
    btStop();
}

void init_ignitor()
{
    pinMode(IGNITOR_REFERENCE_PIN, OUTPUT);
    digitalWrite(IGNITOR_REFERENCE_PIN, HIGH);

    if (ignitor_consistency() < IGNITOR_THRESHOLD)
    {
        #ifdef DEBUG
        Serial.println("Ignitor init failed!");
        #endif
        blink_LED(LED_IGNITOR_ERROR);
    }
    else
    {
        pinMode(IGNITOR_PIN, OUTPUT);
        turn_on_LED();
        #ifdef DEBUG
        Serial.println("Ignitor init OK");
        #endif
    }
}


void init_SPIFFS()
{
    if(!SPIFFS.begin(true))
    {
        #ifdef DEBUG
        Serial.println("SPIFFS init failed");
        #endif
    }
    else
    {
        #ifdef DEBUG
        Serial.println("SPIFFS init OK");
        #endif
    }
}



// Ignitor functions

void ignite()
{
    #ifdef DEBUG
    Serial.println("Ignition!");
    #endif  
    digitalWrite(IGNITOR_PIN, HIGH);
    delay(IGNITE_DELAY);
    digitalWrite(IGNITOR_PIN, LOW);
    #ifdef DEBUG
    Serial.println("Ignited...");
    #endif  
}


byte ignitor_consistency()
{
    int average_voltage = 0;
    for (int i=0; i < AVG_MEAS_COUNT; i++)
    {
        average_voltage += byte(analogRead(IGNITOR_CONTROL_PIN)/5);
    }
    average_voltage /= AVG_MEAS_COUNT;
    #ifdef DEBUG
    Serial.print("Normalized voltage:\t");
    Serial.println(average_voltage);
    #endif
    return (byte)average_voltage;
}


// Control commands


void set_command(int command):
{
    switch(command)
    {
        case STATE_CONTROL:
        set_control();
        break;
        case STATE_ARMED:
        set_armed();
        break;
        default:
        set_error(STATE_ERROR);
        break;
    };
}


void set_error(byte state)
{
    String message = "Error on state " + String(state);
    send_message(message, state);
}


void set_control()
{
    if (state == STATE_DEFAULT)
    {
        gps_data = get_gps_data();
        send_message(message, gps_data);
        state = STATE_CONTROL;
        #ifdef DEBUG
        Serial.println("Sending message: " + message + " | " + gps_data);
        #endif
    }
    else
    {
        set_error(state);
    }
}


void set_armed()
{
    if (state == STATE_CONTROL)
    {
        String message = "Rocket armed!";
        state = STATE_ARMED;
        send_message(message, state);
        #ifdef DEBUG
        Serial.println("Sending message: " + message + " | " + String(state));
        #endif
    }
    else
    {
        set_error(state);
    }
}


String get_sensors_data()
{
    String ret_str = "";
    ret_str = String(millis()) + ",";
    ret_str = get_gyro_data() + ",";
    ret_str = get_gps_data() + ",";
    ret_str = get_altimeter_data();
    return ret_str;
}


// Flight sequence logic

void wait_for_start()
{
    while(true)
    {
        if ((get_altitude() - land_altitude) > altitude_threshold)
        {
            break;
        }
        else
        {
            sensor_data = get_sensors_data();
            #ifdef DEBUG
            Serial.println(sensor_data);
            #endif
            store_spiffs_data(sensor_data);
        }
    }
}
