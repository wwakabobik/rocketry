/* ***************************************************************************
 * This sketch contains LoRa receiver logic                                  *
 *                                                                           *
 * Sketch uses Arduino UNO controller.                                       *
 *                                                                           *
 * Receiver contains:                                                        *
 *    - Arduino UNO R3, piezo buzzer, GY-NEO6MV3 GPS,                        *
 *      7805 stabilizer, SX1278 (Ra-02) LoRa module.                         *
 *                                                                           *
 * Third-party libraries:                                                    *
 *    - https://github.com/mikalhart/TinyGPS                                 *
 *                                                                           *
 * Logic:                                                                    *
 *    1) Init all modules                                                    *
 *    2) Wait until GPS starts and connection established;                   *
 *    3) Print current position                                              *
 *    4) Loop until LoRa packet from flight controller will be received      *
 *    5) Print packet RSSI, beep                                             *
 *    6) Go to 4                                                             *
 *                                                                           *
 * Sketch written by Iliya Vereshchagin 2018.                                *
 *****************************************************************************/

#include <SPI.h>
#include <SoftwareSerial.h>
#include <LoRa.h>
#include <TinyGPS++.h>

//Serial globals
const int BaudRate = 9600;

// Delay Globals
const unsigned long gps_delay = 120000;
const int standard_delay = 500;

// GPS globals
const int GPS_RX = 0, GPS_TX = 1;
SoftwareSerial gps_connection(GPS_RX, GPS_TX);
TinyGPSPlus gps;

// LoRa globals
const int LoRaTxPower = 20;

// buzzer globals;
const int PIN_BUZZER = 6;
const int frequency = 5000;

// Markers
bool marker = false;

/* Init functions */

void initGPS()
{
    gps_connection.begin(BaudRate);
    Serial.println("GPS Start");
    delay(gps_delay);
    while(gps_connection.available() > 0) 
    {
        char temp = gps_connection.read();
        //Serial.write(temp); // debug
        gps.encode(temp);
    }
    Serial.print("GPS timestamp: ");
    Serial.println(String(gps.date.year()) + "-" + String(gps.date.month()) + "-" + String(gps.date.day()) + "," + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()));
    Serial.print("Current position: ");
    Serial.println(String(gps.location.lat()) + "," + String(gps.location.lng()) + "," + String(gps.altitude.meters()));
    Serial.println("GPS OK");
    beep(frequency);
}


void initLoRa()
{
    if (!LoRa.begin(433E6))
    {
        Serial.println("Starting LoRa failed!");
        while(1);
    }
    LoRa.setTxPower(LoRaTxPower);
    Serial.println("LoRa started!");
    beep(frequency);  
}


void initBuzzer()
{
    pinMode(PIN_BUZZER, INPUT);
    Serial.println("Buzzer pin set");
    beep(frequency);
}


void setup() {
    Serial.begin(BaudRate);
    while (!Serial);
    Serial.println("INIT START");
    initBuzzer();
    initGPS();
    initLoRa();
    Serial.println("Now listening...");
}

void loop() 
{
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) 
  {
      // received a packet
      Serial.print("Received packet '");

      // read packet
      while (LoRa.available()) 
      {
          Serial.print((char)LoRa.read());
      }

      // print RSSI of packet
      Serial.print("' with RSSI ");
      Serial.println(LoRa.packetRssi());
    
      if (!marker)
      {
          marker = true;
          for (int i=0; i<3; i++)
          {
              beep(frequency);  
          }
      }
  }
}

void beep(int ghz)
{
    tone(PIN_BUZZER, ghz, standard_delay);
    delay(standard_delay);
    pinMode(PIN_BUZZER, INPUT);
    delay(standard_delay);
}
