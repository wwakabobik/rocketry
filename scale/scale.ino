/* ***************************************************************************
 * This sketch contains scale logic for candy rocket engine test stand.      *
 *                                                                           *
 * Sketch uses Arduino UNO controller, data logger shield, LCD 1602 and      *
 * HX711 module, to measure high-power thrust I use 20kg tensiometer.        *
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
 * Sketch written by Iliya Vereshchagin 2018.                                *
 *****************************************************************************/

#include <SD.h>
#include <LiquidCrystal.h>
#include <HX711.h>
#include <RTClib.h>

//delay globals
const int standard_delay = 1000;
const int short_delay = 100;
const int error_delay = 5000;
const int LCD_delay = 100;

// HX711 globals
HX711 scale;
const float calibration_factor = 102.58; // obtained by scale_calibration
float scale_result;
 
// SD CARD globals
File myFile;

// LCD globals
// initialize the library by associating any needed LCD interface pin
// with the Arduino pin number it is connected to
const int rs = 9, en = 8, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// RTC global
RTC_DS1307 RTC;
DateTime now;
DateTime check;

// output globals 
int counter;
String message;

void setup() 
{
    bool result = false;
    counter=0;
    // start debug
    Serial.begin(9600);

    initRTC();
    initLCD();
    initScale();
    result = initSDCard();
    
    if (result)
    {
        stop();
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
    // update LED not so frequent as measure to avoid blinking
    if (counter%LCD_delay==0)
    {
       lcd.clear();
       lcd.setCursor(14, 0);
       lcd.print(String((check.unixtime()-now.unixtime())));
       lcd.setCursor(0, 1);
       lcd.print(String(scale_result));
       counter=0;
    }
    else
    {
       counter++;
    }
    //generate output
    message = String(millis()) + "," + String(scale_result) + "," + String(check.unixtime()) +"," + String(check.unixtime()-now.unixtime());
    Serial.println(message);
    myFile.println(message);
    // check measurement timeout, if so, close file
    if ((check.unixtime()-now.unixtime())>60)
    {
        myFile.close();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Measurement stop!");
        //Serial.println("Measurement stop!");
        delay(error_delay);
        lcd.clear();
        stop();
    }
}

void stop()
{
    while(1);
}

void startloop()
{
    myFile = SD.open("scale.txt", FILE_WRITE);
    delay(standard_delay);
    // if the file opened okay, start looping:
    if (myFile) 
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Start measure!   ");
        //Serial.println("Start measure!");
        delay(error_delay);    
        myFile.println();
        myFile.println(getTimeStamp());
    }
    else
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Can't open file! ");
        Serial.println("Can't open file!");
        delay(error_delay);
        lcd.clear();
        //pinMode(backLight, OUTPUT);
        //digitalWrite(backLight, LOW); 
        stop();
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

void initLCD()
{
    lcd.begin(16, 2);
    // Print a message to the LCD
    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print("Hello!           ");
    //Serial.println("Hello!");
    delay(standard_delay);
    Serial.println(getTimeStamp());
    lcd.setCursor(0, 1);
    lcd.print(getTimeStamp());
    delay(standard_delay);    
}

void initScale()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Initializing     ");
    lcd.setCursor(0, 1);
    lcd.print("scale...         ");
    //Serial.println("Init scale...");
    scale.begin(A1, A0);
    delay(standard_delay);
    scale.set_scale();
    //Serial.println("Resetting tare...");
    delay(short_delay);
    scale.tare(); // reset to 0
    //Serial.println("Set calibration... factor " + String(calibration_factor));
    delay(short_delay);
    scale.set_scale(calibration_factor); // apply calibration
}

bool initSDCard()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Initializing     ");
    lcd.setCursor(0, 1);
    lcd.print("SD card...       ");
    //Serial.println("Initializing SD card...");
    delay(standard_delay);
    if (!SD.begin(10)) 
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Init SD failed! ");
        Serial.println("Init SD failed!");
        delay(error_delay);
        lcd.clear();
        //pinMode(backLight, OUTPUT);
        //digitalWrite(backLight, LOW); 
        return true;
    }
    //Serial.println("Opening IO file...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.println("Init done         ");
    lcd.setCursor(0, 1);
    lcd.println("opening IO file...");
    myFile = SD.open("scale.txt", FILE_WRITE);
    delay(standard_delay);
    // if the file opened, return false
    if (myFile)
    {
        myFile.close();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Init OK.          ");
        lcd.setCursor(0, 1);
        lcd.print("Scale ready!      ");
        //Serial.println("Init OK. Scale ready!");
        delay(standard_delay);
        return false;
    }
    else
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Can't open file!  ");
        Serial.println("Can't open file!");
        delay(error_delay);
        lcd.clear();
        //pinMode(backLight, OUTPUT);
        //digitalWrite(backLight, LOW); 
        return true;
    }
}

