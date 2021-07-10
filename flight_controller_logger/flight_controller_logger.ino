/* ***************************************************************************
 * This sketch contains flight controller logic for candy rockets.           *
 *                                                                           *
 * Sketch uses Arduino Nano controller, and it contains limited amount of    *
 * RAM. Due to that - to achieve stability - at least 20% of RAM should be   *
 * free and debug serial output is commented-out in this sketch.             *
 *                                                                           *
 * Flight controller contains:                                               *
 *    - Arduino Nano v3 (CH340g), MPU-6050 6-axis gyroscope, piezo buzzer,   *
 *      BMP180 barometer/thermometer, SD card module, 2x18650, LM7805, relay *
 *                                                                           *
 * Third-party libraries:                                                    *
 *    - https://github.com/sparkfun/BMP180_Breakout                          *
 *                                                                           *
 * Logic:                                                                    *
 *    1) Init all modules;                                                   *
 *    2) Wait until SD card ready;                                           *
 *    3) Wait for jumper out or altitude reach limit change (5-10m);         *
*     4) Start looping, write gyro and altimeter data to file;               *
*     5) If altitude starts to falling or timer is out, ignite;              *
*     6) If time after ignition reach landing limit, Stop loop, save file;   *
 *    7) Beep continuously until vessel will be recovered.                   *
 *                                                                           *
 * Sketch written by Iliya Vereshchagin 2021.                                *
 *****************************************************************************/

//#define DEBUG
//#define FAKE_RUN
//#define JUMPER_START
#define ALTITUDE_START

#include <SD.h>
#include <Wire.h>
#include <SFE_BMP180.h>
#include <SoftwareSerial.h>
#include <SPI.h>

//Serial globals
const int BaudRate = 9600;

// delay globals
const int standard_delay = 500;
const int error_delay = 100;
const long flight_delay = 10000;
const long apogee_delay = flight_delay / 2;
const long landing_delay = flight_delay * 12;

// Gyro globals
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
// Barometer globals
SFE_BMP180 barometer_data;
double normal_pressure = 0;
double altitude = 0;
double max_altitude = 0;
const double max_delta = 5.0;

// SD globals
const String filename = "FDATA.TXT";
const int SD_CS_PIN = 5;
File myFile;

// buzzer globals;
const int PIN_BUZZER = 6;
const int frequency = 5000;

// jumper globals
const int PIN_JUMPER = 2;

// ignition globals
const int PIN_IGNITOR = 9;
bool ignited = false;

// landing globals
unsigned long start_time;


void setup()
{
    #ifdef DEBUG
    Serial.begin(BaudRate);
    #endif
    initBuzzer();
    initBarometer();
    initGyro();
    initIgnitor();
    initSDCard();
    altitude = getRawBarometerData(2);
    beep(frequency, standard_delay); // make beep, mark init completed
    #ifdef DEBUG
    Serial.println("Init completed, waiting for rocket start");
    #endif
    wait_for_jumper();
}


void wait_for_jumper()
{
    double delta_altitude;
    while(1)
    {
        #ifdef ALTITUDE_START
        delta_altitude = getRawBarometerData(2) - altitude;
        if (delta_altitude > max_delta)
        {
            #ifdef DEBUG
            Serial.println("Rocket started, forcing loop");
            #endif
            break;
        }
        // failsafe: are we rebooting in flight? If yes, ignite immidiately
        if ((-delta_altitude) > (max_delta * 2))
        {
            #ifdef DEBUG
            Serial.println("Rocket falling! Urgent ignition!");
            #endif
            ignite();  // try to eject chute
            stopFlight(); // close file anyway
            rescueBeep(standard_delay); // start beeping
        }
        #endif
        #ifdef JUMPER_START
        if (digitalRead(PIN_JUMPER) == LOW)
        {
            #ifdef DEBUG
            Serial.println("Jumper out, forcing loop");
            #endif
            break;
        }
        break;
        #endif
        #ifdef FAKE_RUN
        fake_ignition();
        break;
        #endif
    }
    start_time = millis();
}

/* Init functions */

void initIgnitor()
{
    #ifdef DEBUG
    Serial.println("Initializing ignitor pin...");
    #endif
    pinMode(PIN_IGNITOR, OUTPUT);
    digitalWrite(PIN_IGNITOR, HIGH);
}

void initJumper()
{
    #ifdef DEBUG
    Serial.println("Initializing jumper pin...");
    #endif
    pinMode(PIN_JUMPER, INPUT_PULLUP); 
}


void initGyro()
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    #ifdef DEBUG
    Serial.println("Gyro start");
    #endif
}


bool initSDCard()
{
    #ifdef DEBUG
    Serial.println("Initializing SD card...");
    #endif
    delay(standard_delay);
    if (!SD.begin(SD_CS_PIN)) 
    {
        #ifdef DEBUG
        Serial.println("Init SD failed!");
        #endif
        stop();
    }
    #ifdef DEBUG
    Serial.println("Opening IO file...");
    #endif
    myFile = SD.open(filename, FILE_WRITE);
    delay(standard_delay);
    // if the file opened, return false
    if (myFile)
    {
        #ifdef DEBUG
        Serial.println("SD ready");
        #endif
        delay(standard_delay);
    }
    else
    {
        #ifdef DEBUG
        Serial.println("Can't open file!");
        #endif
        stop();
    }
}


void initBarometer()
{
    barometer_data.begin();
    normal_pressure = getRawBarometerData(0);
    #ifdef DEBUG
    Serial.println("Barometer set");
    #endif
}


void initBuzzer()
{
    pinMode(PIN_BUZZER, INPUT);
    #ifdef DEBUG
    Serial.println("Buzzer pin set");
    #endif
}


/* Execution functions */


void writeFlightData()
{
    myFile.print(millis());
    #ifdef DEBUG
    Serial.print(millis());
    #endif
    myFile.print(",");
    #ifdef DEBUG
    Serial.print(",");
    #endif
    myFile.print(getGyroData());
    #ifdef DEBUG
    Serial.print(getGyroData());
    #endif
    myFile.print(",");
    #ifdef DEBUG
    Serial.print(",");
    #endif
    myFile.print(getBarometerData());
    #ifdef DEBUG
    Serial.print(getBarometerData());
    #endif
    myFile.print("\n");
    #ifdef DEBUG
    Serial.print("\n");
    #endif
}


String getGyroData()
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    String retVal = String(AcX) + "," + String(AcY) + "," + String(AcZ) + "," + String(Tmp/340.00+36.53) + "," + String(GyX) + "," + String(GyY) + "," + String(GyZ);
    return retVal;
}


String getBarometerData()
{   
    altitude = getRawBarometerData(2);
    String retVal = String(getRawBarometerData(0)) + "," + String(getRawBarometerData(1)) + "," + String(altitude);
    return retVal;
}


double getRawBarometerData(int type)
{
    char status;
    double T, P;

    status = barometer_data.startTemperature();
    if (status != 0)
    {
        delay(status);
        status = barometer_data.getTemperature(T);
        if (type == 1)
        {
            return T;
        }
        if (status != 0)
        {
            status = barometer_data.startPressure(3);
            if (status != 0)
            {
                delay(status);
                status = barometer_data.getPressure(P, T);
                if (status != 0)
                {
                    if (type == 0)
                    {
                        return(P);
                    }
                    else
                    {
                        return barometer_data.altitude(P, normal_pressure);
                    }
                }
            }
        }
    } 
}


void loop()
{
    // write flight data every tick  
    writeFlightData();

    // write new apogee
    if (altitude > max_altitude)
    {
        max_altitude = altitude;
    }

    // check for apogee
    if ((max_altitude - altitude) > max_delta and ignited == false and ((millis() - start_time) > apogee_delay))
    {
        #ifdef DEBUG
        Serial.println("Apogee reached!");
        #endif
        ignite();
    }

    // check that time limit is reached
    if (((millis() - start_time) > flight_delay) and ignited == false)
    {
        #ifdef DEBUG
        Serial.println("Time limit reached!");
        #endif
        ignite();
    }

    // check for flight end
    if ((millis() - start_time) > landing_delay)
    {
        // close file if it's opened
        if (myFile)
        {
            stopFlight();
        }
        rescueBeep(standard_delay);
    }
}


void stopFlight()
{
    #ifdef DEBUG
    Serial.println("Landed, saving data and call for recovery");
    #endif
    myFile.close();
    digitalWrite(PIN_IGNITOR, HIGH);
}


void stop()
{
    while(1)
    {
        beep(frequency, error_delay);
        delay(error_delay);
    }
}


void ignite()
{
    #ifdef DEBUG
    Serial.println("Deploying chute, ignition!");
    #endif
    digitalWrite(PIN_IGNITOR, LOW);
    ignited = true;
}


/* buzzer functions */
void rescueBeep(int demanded_delay)
{
    beep(frequency, demanded_delay);
    delay(demanded_delay);
}


void beep(int ghz, int demanded_delay)
{
    tone(PIN_BUZZER, ghz, demanded_delay);
    delay(demanded_delay);
    pinMode(PIN_BUZZER, INPUT);
    delay(demanded_delay);
}


#ifdef FAKE_RUN
void fake_ignition()
{
    start_time = millis();
    delay(flight_delay);
    beep(frequency, standard_delay);
}
#endif
