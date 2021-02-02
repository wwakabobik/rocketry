/* ***************************************************************************
 * This sketch contains scale logic for candy rocket engine test stand.      *
 *                                                                           *
 * Sketch uses Arduino UNO controller, data logger shield, LEDs and          *
 * HX711 module, to measure high-power thrust I use 20kg tensiometer.        *
 *                                                                           *
 * Third-party libraries:                                                    *
 *    - https://github.com/bogde/HX711.git                                   *
 *    - https://github.com/adafruit/RTClib.git                               *
 *                                                                           *
 * Logic:                                                                    *
 *    1) Init all modules (HX711, RTC, SD card)                              *
 *    2) Log meaurement data to SD card every tick (approx exery 0,1 sec)    *
 *    3) Stop measurement                                                    *
 *                                                                           *
 * Sketch written by Iliya Vereshchagin 2019.                                *
 *****************************************************************************/

#include <SD.h>
#include <HX711.h>
#include <RTClib.h>

// Delay globals
const int standard_delay = 1000;
const int short_delay = 100;

// LED globals
const int LED_red = 7;
const int LED_green = 6;

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
    Serial.begin(9600);

    initLED();
    initRTC();
    initScale();
    
    if (initSDCard())
    {
        sd_error();
    }
    startloop();
    check = RTC.now();
}

void loop() 
{
    // get scale data
    scale_result=scale.get_units(1);
    // get current time
    check = RTC.now();
    //generate output
    message = String(millis()) + "," + String(scale_result) + "," + String(check.unixtime()) +"," + String(check.unixtime()-now.unixtime());
    Serial.println(message);
    myFile.println(message);
    // check measurement timeout, if so, close file
    if ((check.unixtime()-now.unixtime())>60)
    {
        myFile.close();
        Serial.println("Measurement stop!");
        delay(standard_delay);
        stop_measure();
    }
}

void countdown()
{
    int counter = 5;
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
    while(1);
}

void startloop()
{
    myFile = SD.open("scale.txt", FILE_WRITE);
    delay(standard_delay);
    if (myFile) 
    {
        countdown();
        Serial.println("Start measure!"); 
        myFile.println();
        myFile.println(getTimeStamp());
    }
    else
    {
        Serial.println("Can't open file!");
        delay(standard_delay);
        sd_error();
    }
}

String getTimeStamp()
{
    now = RTC.now();
    return String(now.year(), DEC) + "/" + String(now.month(), DEC) + "/" + String(now.day(), DEC) + " " + String(now.hour(), DEC) + ":" + String(now.minute(), DEC);
}

void initRTC()
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

void initScale()
{
    //Serial.println("Init scale...");
    scale.begin(A1, A0);
    delay(standard_delay);
    scale.set_scale();
    Serial.println("Resetting tare...");
    delay(short_delay);
    scale.tare(); // reset to 0
    Serial.println("Set calibration... factor " + String(calibration_factor));
    delay(short_delay);
    scale.set_scale(calibration_factor); // apply calibration
}

bool initSDCard()
{
    //Serial.println("Initializing SD card...");
    delay(standard_delay);
    if (!SD.begin(10)) 
    {
        Serial.println("Init SD failed!");
        delay(standard_delay);
        return true;
    }
    Serial.println("Opening IO file...");
    myFile = SD.open("scale.txt", FILE_WRITE);
    delay(standard_delay);
    if (myFile)
    {
        myFile.close();
        Serial.println("Init OK. Scale ready!");
        delay(standard_delay);
        return false;
    }
    else
    {
        Serial.println("Can't open file!");
        delay(standard_delay);
        return true;
    }
}

void initLED()
{
    pinMode(LED_green, OUTPUT);
    pinMode(LED_red, OUTPUT);
    digitalWrite(LED_red, HIGH);
}
