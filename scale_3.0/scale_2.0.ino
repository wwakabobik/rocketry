/* ***************************************************************************
 * This sketch contains scale logic for candy rocket engine test stand.      *
 *                                                                           *
 * Sketch uses Arduino UNO controller, data logger shield, LEDs, relay       *
 * and HX711 module, to measure high-power thrust I use 50kg tensiometer.    *
 *                                                                           *
 * Third-party libraries:                                                    *
 *    - https://github.com/bogde/HX711.git                                   *
 *    - https://github.com/adafruit/RTClib.git                               *
 *                                                                           *
 * Logic:                                                                    *
 *    1) Init all modules (HX711, RTC, SD card)                              *
 *    2) Log measurement data to SD card every tick (approx every 0,1 sec)   *
 *    3) Stop measurement                                                    *
 *                                                                           *
 * Sketch written by Iliya Vereshchagin 2021.                                *
 *****************************************************************************/

#define DEBUG

#include <SD.h>
#include <HX711.h>
#include <RTClib.h>

// Delay globals
const int standard_delay = 1000;
const int short_delay = 100;
const int counter_seconds = 60;
const int measurement_time = 30;

// LED globals
const int LED_red = 7;
const int LED_green = 6;

// ignitor globals
const int ignitor_PIN = 9;

// HX711 globals
HX711 scale;
const float calibration_factor = 126.53; // obtained by scale_calibration
float scale_result;
 
// SD CARD globals
File myFile;

// RTC global
RTC_DS1307 RTC;
DateTime now;
DateTime check;

// output globals 
String message;

void setup() 
{
    #ifdef DEBUG
    Serial.begin(9600);
    #endif

    init_LED();
    init_RTC();
    init_scale();
    init_relay()

    
    if (init_SD_card())
    {
        sd_error();
    }
    start_loop();
    check = RTC.now();
}

void loop() 
{
    // get scale data
    scale_result=scale.get_units(1);
    // get current time
    check = RTC.now();
    //generate output
    message = String(millis()) + "," + String(scale_result) + ",";
    message += String(check.unixtime()) + "," + String(check.unixtime()-now.unixtime());
    #ifdef DEBUG
    Serial.println(message);
    #endif
    myFile.println(message);
    // check measurement timeout, if so, close file
    if ((check.unixtime()-now.unixtime()) > measurement_time)
    {
        myFile.close();
        #ifdef DEBUG
        Serial.println("Measurement stop!");
        #ifdef endif
        delay(standard_delay);
        stop_measure();
    }
}

void countdown()
{
    int counter = counter_seconds;
    while(counter > 0)
    {
        digitalWrite(LED_green, HIGH);
        digitalWrite(LED_red, LOW);
        delay(standard_delay/2);
        digitalWrite(LED_green, LOW);
        digitalWrite(LED_red, HIGH);
        delay(standard_delay/2);
        counter = counter - 1;
    }
    digitalWrite(LED_green, HIGH);
    digitalWrite(LED_red, HIGH);
    delay(standard_delay);
}

void sd_error()
{
    digitalWrite(LED_green, LOW);
    while(1)
    {
        if (digitalRead(LED_red) == LOW)
        {
            digitalWrite(LED_red, HIGH);
        }
        else
        {
            digitalWrite(LED_red, LOW);
        }
        delay(standard_delay);
    }
}

void stop_measure()
{
    digitalWrite(LED_green, LOW);
    digitalWrite(ignitor_PIN, LOW);
    while(1);
}

void start_loop()
{
    myFile = SD.open("scale.txt", FILE_WRITE);
    delay(standard_delay);
    if (myFile) 
    {
        countdown();
        #ifdef DEBUG
        Serial.println("Start measure!");
        #endif
        myFile.println();
        myFile.println(get_time_stamp());
        digitalWrite(ignitor_PIN, HIGH);
    }
    else
    {
        #ifdef DEBUG
        Serial.println("Can't open file!");
        #endif
        delay(standard_delay);
        sd_error();
    }
}

String get_time_stamp()
{
    now = RTC.now();
    String ret_string = String(now.year(), DEC) + "/" + String(now.month(), DEC) + "/" + String(now.day(), DEC);
    ret_string += " " + String(now.hour(), DEC) + ":" + String(now.minute(), DEC)
    return ret_string;
}

void init_RTC()
{
    RTC.begin();
    if (! RTC.isrunning()) 
    {
        Serial.println("RTC is NOT running!");
        // following line sets the RTC to the date & time this sketch was compiled
        // uncomment it & upload to set the time, date and start run the RTC!
        //RTC.adjust(DateTime(__DATE__, __TIME__));
    }    
}

void init_scale()
{
    //Serial.println("Init scale...");
    scale.begin(A1, A0);
    delay(standard_delay);
    scale.set_scale();
    #ifdef DEBUG
    Serial.println("Resetting tare...");
    #endif
    delay(short_delay);
    scale.tare(); // reset to 0
    #ifdef DEBUG
    Serial.println("Set calibration... factor " + String(calibration_factor));
    #endif
    delay(short_delay);
    scale.set_scale(calibration_factor); // apply calibration
}

bool init_SD_card()
{
    //Serial.println("Initializing SD card...");
    delay(standard_delay);
    if (!SD.begin(10)) 
    {
        Serial.println("Init SD failed!");
        delay(standard_delay);
        return true;
    }
    #ifdef DEBUG
    Serial.println("Opening IO file...");
    #ifdef endif
    myFile = SD.open("scale.txt", FILE_WRITE);
    delay(standard_delay);
    if (myFile)
    {
        myFile.close();
        #ifdef DEBUG
        Serial.println("Init OK. Scale ready!");
        #endif
        delay(standard_delay);
        return false;
    }
    else
    {
        #ifdef DEBUG
        Serial.println("Can't open file!");
        #ifdef endif
        delay(standard_delay);
        return true;
    }
}

void init_LED()
{
    pinMode(LED_green, OUTPUT);
    pinMode(LED_red, OUTPUT);
    digitalWrite(LED_red, HIGH);
}

void init_ignitor()
{
    pinMode(ignitor_PIN, OUTPUT);
    digitalWrite(ignitor_PIN, LOW);
}
