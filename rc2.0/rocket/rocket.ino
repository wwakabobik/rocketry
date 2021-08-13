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

//#define DEBUG

#include <Arduino.h>

#include <SPI.h>              
#include <LoRa.h>
#include <WiFi.h>
#include "FS.h"
#include "SPIFFS.h"

#include "config.h"
#include "gps.h"
#include "beeper.h"
#include "lora_module.h"
#include "gyro.h"
#include "altimeter.h"


// SPIFFS globals

// LoRa communication
int state = STATE_DEFAULT;
String message = "";


void setup()
{
    #ifdef ROCKET_DEBUG
    Serial.begin(9600);
    #endif
    init_communication()
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
    pinMode(IGNITOR_PIN, OUTPUT);
    #ifdef ROCKET_DEBUG
    Serial.println("Ignitor init OK");
    #endif
}


void init_SPIFFS()
{
    if(!SPIFFS.begin(true))
    {
        #ifdef ROCKET_DEBUG
        Serial.println("SPIFFS init failed");
        #endif
    }
    else
    {
        #ifdef ROCKET_DEBUG
        Serial.println("SPIFFS init OK");
        #endif
    }
}



// Ignitor functions

void ignite()
{
    #ifdef ROCKET_DEBUG
    Serial.println("Ignition!");
    #endif  
    digitalWrite(IGNITOR_PIN, HIGH);
    #ifdef ROCKET_DEBUG
    Serial.println("Ignited...");
    #endif  
}


// Control commands


void set_command(int command)
{
    switch(command)
    {
        case STATE_ROCKET_CONTROL:
        set_control();
        break;
        case STATE_ROCKET_ARMED:
        set_armed();
        break;
        default:
        set_error(STATE_ROCKET_ERROR);
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
        message = get_GPS_data();
        send_message(message, 0x88);
        state = STATE_ROCKET_CONTROL;
        #ifdef ROCKET_DEBUG
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
    if (state == STATE_ROCKET_CONTROL)
    {
        String message = "Rocket armed!";
        state = STATE_ROCKET_ARMED;
        send_message(message, state);
        #ifdef ROCKET_DEBUG
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
    ret_str = get_GPS_data() + ",";
    ret_str = get_altimeter_data();
    return ret_str;
}


// Flight sequence logic

void wait_for_start()
{
    while(true)
    {
        if ((get_altitude() - get_land_altitude()) > altitude_threshold)
        {
            break;
        }
        else
        {
            String sensor_data = get_sensors_data();
            #ifdef ROCKET_DEBUG
            Serial.println(sensor_data);
            #endif
            //store_spiffs_data(sensor_data);
        }
    }
}
