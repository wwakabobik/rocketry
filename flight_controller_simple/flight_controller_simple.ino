/* ***************************************************************************
 * This sketch contains simple flight controller logic for candy rockets.    *
 *                                                                           *
 * Sketch uses Arduino Nano controller, and it contains limited amount of    *
 * RAM. Due to that - to achieve stability - at least 20% of RAM should be   *
 * free.                                                                     *
 *                                                                           *
 * Flight controller contains:                                               *
 *    - Arduino Nano v3 (CH340g), piezo buzzer, 9v battery, 2x18650 battery, *
 *      BMP180 barometer/thermometer, KY-017 tilt sensor (or SW-420),        *
 *      two power power relay switches, two LM393 luminosity sensors, LED.   *
 *      It's better to use PC jumper to init start, and LM7805 stabilizer.   *
 *                                                                           *
 *      Note: all or only one sensor/timer can be used at same time.         *
 *      Choice is up to you. Best screnario is to trigger ignition at apogee *
 *      Most precise way is to use altimeter with minimum altitude trigger   *
 *      difference. Most robustness way - is to use tilt sensor in pair with *
 *      timer.                                                               *
 *                                                                           *
 * Third-party libraries:                                                    *
 *    - https://github.com/adafruit/Adafruit-BMP085-Library                  *
 *                                                                           *
 * Logic:                                                                    *
 *    1) Init all modules                                                    *
 *    2) Loop until jumper out                                               *
 *    4) Check for any sensor triggered, or timer is up                      *
 *    5) Trigger ignitor relay on                                            *
 *    6) Beep until rescue                                                   *
 *                                                                           *
 * Sketch written by Iliya Vereshchagin 2020.                                *
 *****************************************************************************/

#include <Adafruit_BMP085.h>


/* Globals */

// Pin globals
const int buzzerPin = 2;
const int ignitionPin = 3;
const int inPin = 4;
const int ledPin = 5;
const int tiltPin = 6;
const int luminosityUpperPin = 7;
const int luminosityBottomPin = 8;

// Delay globals
const int startDelay = 30000;  // delay until system on standby, it's highly recommended to set up at least to 30 secs
const int deployDelay = 8000;  // delay from jumper out to force eject chute
const int ignitionDelay = 10000;  // length of ignition, for 9v battery it may take up to 5-6 seconds to warm up, for 18650 is about 1-2 secs
const int toneDelay = 500;  //  delay between rescue beeps / beeps length 

// Buzzer globals
const int toneHz = 520;  // just buzzer noise tone in hz

// Serial globals
const int serialSpeed = 9600;

// Altitude sensor
Adafruit_BMP085 BMP180;  // BMP180 sensor object
double maxAltitude;  // maximum altitude
double seaLevelPressure_hPa;  // current pressure at start point
double dropLimit = 2.0;  // maximum fall to force eject in meters 

// Timer globals
int startTime;


/* Main functions */

void setup() 
{
    Serial.begin(serialSpeed);
    initIgnition();
    initJumper();
    initLed();
    initBuzzer();
    initLuminositySensors();
    initTiltSensor();
    initAltitudeSensor();
    beep(tone, toneDelay);
    delay(startDelay);
    beep(tone, toneDelay);
    ledOn();
}

void loop() 
{
    wait_for_jumper();
}


/* Main routine functions */

void wait_for_jumper()
{
    // Check jumper state
    if (digitalRead(inPin)==LOW)
    {
        Serial.println("Jumper out!");
        ledOff();
        startTime = millis();
        Serial.println("Wait 8 secs...");
        while (!checkTimer)
        {
            // Comment-out not used sensors
            if (checkTiltSensor())
            {
                break;
            }
            
            if (checkLuminositySensor())
            {
                break;
            }

            if (checkDifferenceSensor())
            {
                break;
            }

            if (checkAltitudeSensor())
            {
                break;
            } 
        }
        eject();
        rescue();
    }
}

void eject()
{
    Serial.println("Deploy chute!");
    digitalWrite(ignitionPin, HIGH);
    delay(ignitionDelay);
    digitalWrite(ignitionPin, LOW);
}

void rescue()
{
    Serial.println("Call for rescue...");
    while(1)
    {
        beep(toneHz, toneDelay);
    }
}


/* Init functions */

void initLed()
{
    pinMode(inPin, OUTPUT);
    ledOff();
    Serial.println("Led init success");
}

void initBuzzer()
{
  pinMode(buzzerPin, INPUT);
  Serial.println("Buzzer init success");
}

void initIgnition()
{
    pinMode(ignitionPin, OUTPUT);
    digitalWrite(ignitionPin, LOW);
    Serial.println("Ignition pin init success");
}

void initJumper()
{
    pinMode(inPin, INPUT);
    Serial.println("Jumper init success");
}

void initTiltSensor()
{
    pinMode(tiltPin, INPUT);
    // Check that reading is ok
    if (checkTiltSensor())
    {
        // sensor should not be activated
        Serial.println("Tilt sensor init fail");
        rescue();
    }
    else
    {
        Serial.println("Tilt sensor init success");
    }
}

void initLuminositySensors()
{
    pinMode(luminosityUpperPin, INPUT);
    pinMode(luminosityBottomPin, INPUT);
    // Check that readings is ok
    if (checkLuminositySensor() || checkDifferenceSensor())
    {
        // sensor should not be activated
        Serial.println("Luminocity sensor init fail");
        rescue();
    }
    else
    {
        Serial.println("Luminocity sensors init success");
    }
}

void initAltitudeSensor()
{
    double currentAltitude;
    int success;
    if (BMP180.begin())
    {
        seaLevelPressure_hPa = BMP180.readPressure();
        currentAltitude = BMP180.readAltitude();
        Serial.println("Current pressure is " + String(seaLevelPressure_hPa) + " at " + String(currentAltitude) + " meters");
        Serial.println("Altitude sensor init success");
    }
    else
    {
        Serial.println("Altitude sensor init fail");
        rescue();
    }
}


/*  Check sensors functions */

bool checkTimer()
{
    if (millis() - startTime > deployDelay)
    {
       Serial.println("Time is up");
       return true;
    }
    else
    {
       return false;
    }
}

bool checkTiltSensor()
{
    if (digitalRead(tiltPin)==HIGH)
    {
        Serial.println("Rocket tilt happened");
        return true;
    }
    else
    {
        return false;
    }    
}

bool checkLuminositySensor()
{
    if (digitalRead(luminosityUpperPin)==LOW)
    {
        Serial.println("The skies are bellow the fins");
        return true;
    }
    else
    {
        return false;
    }
}

bool checkDifferenceSensor()
{
    if (digitalRead(luminosityBottomPin)==HIGH && digitalRead(luminosityUpperPin)==LOW)
    {
        Serial.println("Rocket roll over");
        return true;
    }
    else
    {
        return false;
    }    
}

bool checkAltitudeSensor()
{
    double currentAltitude;
    currentAltitude = BMP180.readAltitude(seaLevelPressure_hPa * 100);
    if (currentAltitude > maxAltitude)
    {
        maxAltitude = currentAltitude;
    }
    else
    {
        if (maxAltitude - currentAltitude > dropLimit)
        {
            Serial.println("Maximum altitude reached at " + String(maxAltitude) + " meters");
            return true;
        }
    }
}


/* LED control */

void ledOn()
{
    digitalWrite(ledPin, HIGH);
}

void ledOff()
{
    digitalWrite(ledPin, LOW);
}


/* Beep function */

void beep(int note, int duration)
{
    //Play tone on buzzerPin
    tone(buzzerPin, note, duration);
    delay(duration);
    //Stop tone on buzzerPin
    pinMode(buzzerPin, INPUT);
    delay(toneDelay);
}
