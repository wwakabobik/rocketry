/* ***************************************************************************
 * This sketch contains logic of reciever for ignitor                        *
 *                                                                           *
 * Sketch uses Arduino Nano controller, NRF24L01 module, piezo buzzer,       *
 * relay module and 2x18650                                                  *
 *                                                                           *
 * Third-party libraries:                                                    *
 *  - https://github.com/oevsegneev/arduino-dev/tree/master/libs/SerialFlow  *
 *  - http://www.airspayce.com/mikem/arduino/RadioHead/RadioHead-1.41.zip    *
 *                                                                           *
 * Logic:                                                                    *
 *    1) Init LED, button and Wi-fi (RF433, LoRA)                            *
 *    2) Wait for button press or recieving magic keyword                    *
 *    3) If button pressed or magic keyword activated, start 5-sec countdown *
 *    4) Ignite                                                              *
 *                                                                           *
 * Sketch written by Iliya Vereshchagin 2018. Updated on 2020.               *
 *****************************************************************************/

#include <SerialFlow.h>
#include <RH_ASK.h>      // for RF433
#include <SPI.h>         // for RF433
#include <LoRa.h>        // for LoRA

//pin consts
const int buzzerPin = 4;
const int ledPin = 5;
const int powerPin = 6;
const int buttonPin = 2;

//delay const
const int tone_delay = 500;
const int sequence_delay = 500;
const int ignition_delay = 3000;
const int magic_word = 4242;

//serial flow settings
SerialFlow rd(9,10);

// RF433
RH_ASK RF_driver;
 
int counter = 0;
 
void setup()
{
    //start debug output
    Serial.begin(9600);
    initPins();
    initWiFi();
    initLoRA();
    //start wait for signal
    turnLEDOn();
    return;
}
 
void loop()
{
    checkButton();
    checkPacket();
    checkRF433();
    checkLoRA();
}
 
void turnLEDOn()
{
    Serial.println("Turn ON standby LED");
    digitalWrite(ledPin, HIGH);  
}

void turnLEDOff()
{
    Serial.println("Turn OFF standby LED");
    digitalWrite(ledPin, LOW);  
}

void finalCountdown()
{
    Serial.println("Countdown...");
    Serial.println("5...");
    beep(1000);
    Serial.println("4...");
    beep(2000);
    Serial.println("3...");
    beep(3000);
    Serial.println("2...");
    beep(4000);
    Serial.println("1...");
    beep(5000);
    Serial.println("0...");
}

void checkButton()
{
    if (digitalRead(buttonPin)==HIGH)
    {
        Serial.println("Button pressed");
        launch();
    }
}

void ignite()
{
    Serial.println("Ignition!");
    digitalWrite(powerPin, HIGH);
    delay(ignition_delay);
    digitalWrite(powerPin, LOW);
    Serial.println("Done");
}

void stop()
{
    Serial.println("Stopped");
    while(1);
}

void checkPacket()
{
    unsigned int data;
    if(rd.receivePacket())
    {
        Serial.println("Recieved packet... decoding...");
        data = rd.getPacketValue(0);
        Serial.println("Message is: " + String(data));
        if (data == magic_word)
        {
            Serial.println("Launch sequence obtained, starting...");
            launch();
        }
    }
}

void checkRF433()
{
    int data;
    if (RF_driver.recv(data, 1))
    {
        Serial.println("Recieved packet... decoding...");
        Serial.println("Message is: " + String(data));
        if (data == magic_word)
        {
            Serial.println("Launch sequence obtained, starting...");
            launch();
        }
}

void checkLoRA()
{
    // try to parse packet
    int data;
    int packetSize = LoRa.parsePacket();
    if (packetSize) 
    {
        // received a packet
        Serial.print("Received packet with size ");
        Serial.print((char)packetSize);
        Serial.print(" : '");

        // read packet
        while (LoRa.available()) 
        {
            data = LoRa.read();
            Serial.print((char)data);
        }

        // print RSSI of packet
        Serial.print("' with RSSI ");
        Serial.println(LoRa.packetRssi());

        // packed valid if length == 1
        if (packetSize == 1 and data == magic_word):
        {
            Serial.println("Launch sequence obtained, starting...");
            launch();  
        }
    }  
}

void launch()
{
    turnLEDOff();
    delay(sequence_delay);
    finalCountdown();
    ignite();
    turnLEDOn();
}

void beep(int ghz)
{
    tone(buzzerPin,ghz,tone_delay);
    delay(tone_delay);
    pinMode(buzzerPin, INPUT);
    delay(tone_delay);
}

void initPins()
{
    //Setup pin modes
    Serial.println("Set pins");
    noTone(buzzerPin);
    pinMode(buzzerPin, INPUT);
    pinMode(ledPin, OUTPUT);
    pinMode(powerPin, OUTPUT);
    pinMode(buttonPin, OUTPUT);
    digitalWrite(buttonPin, LOW);
    digitalWrite(powerPin, LOW);   
    pinMode(buttonPin, INPUT);
}

void initWiFi()
{
    //Setup wi-fi
    Serial.println("Set wi-fi");
    rd.setPacketFormat(2, 1);
    rd.begin(0xF0F0F0F0E1LL,0xF0F0F0F0D2LL);    
}

void initLoRa()
{
    if (!LoRa.begin(433E6))
    {
      Serial.println("Starting LoRa failed!");
      //stop();  // uncomment if needed
    }
    LoRa.setTxPower(20);
    Serial.println("LoRa started!");  
}

