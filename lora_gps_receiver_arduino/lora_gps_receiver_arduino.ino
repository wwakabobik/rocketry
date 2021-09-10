/* ***************************************************************************
 * This sketch contains LoRa gps receiver logic                              *
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
 *    5) Print packet RSSI and message (distance and beep if needed)         *
 *    6) Go to 4                                                             *
 *                                                                           *
 * Sketch written by Iliya Vereshchagin 2021.                                *
 *****************************************************************************/

#include <SPI.h>
#include <SoftwareSerial.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <math.h>

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
const int LORA_POWER = 20;                // set TX power to maximum
const int LORA_RETRIES = 12;              // try to init LoRa several times before error
const int LORA_DELAY = 500;               // delay between retries
const int LORA_SEND_DELAY = 100;          // delay between send data
const int LORA_SEND_RETRIES = 5;          // how much packets will be sent
const byte LORA_LOCAL_ADDRESS = 0xDD;     // address of this device
const byte LORA_IGNITOR = 0xBB;           // destination to send to
const byte LORA_ROCKET = 0xCC;            // destination to send to
const byte LORA_KEYWORD = 0x42;           // verification keyword
const int LORA_RECEIVE_DELAY = 500;       // stop activity if correct packet received
const unsigned long LORA_TIMEOUT = (LORA_RECEIVE_DELAY + (LORA_SEND_DELAY * LORA_SEND_RETRIES * 10));

// buzzer globals;
const int PIN_BUZZER = 6;
const int frequency = 5000;

// encoder globals
const bool CALCULATE_DISTANCE = True;
const int LAT_POS = 11;
const int LNG_POS = 12;

struct LoRa_packet{
  String message;
  byte command;
  byte sender;
  int distance = 0;
} lora_packet;

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


void init_LoRa()  // try to init LoRA at 433Mhz for several retries
{
    bool success = false;
    for (int i=0; i < LORA_RETRIES; i++)
    {
        if (LoRa.begin(433E6))
        {
            success = true;
            break;
        }
        delay(LORA_DELAY);
    }
    if (!success)
    {
        #ifdef DEBUG
        Serial.println("LoRa init failed.");
        #endif
        blink_LED(LED_POWER_PIN);
    }

    LoRa.setTxPower(LORA_POWER);  // amplify TX power
    #ifdef DEBUG
    Serial.println("LoRa started!");
    #endif
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
    get_rocket_telemetry();
}


bool onReceive(int packetSize)
{
    if (packetSize == 0)
    {
        return false;                           // if there's no packet, return
    }

    // read packet header bytes:
    int recipient = LoRa.read();          // recipient address
    byte sender = LoRa.read();            // sender address
    byte magic_word = LoRa.read();        // incoming msg ID
    byte command = LoRa.read();           // command
    byte incomingLength = LoRa.read();    // incoming msg length

    String incoming = "";

    while (LoRa.available())
    {
        incoming += (char)LoRa.read();
    }

    if (incomingLength != incoming.length())
    {
        // check length for error
        #ifdef DEBUG
        Serial.println("error: message length does not match length");
        #endif
        return false;
    }

    if (magic_word != LORA_KEYWORD)
    {
        #ifdef DEBUG
        Serial.println("Wrong magic word, aborted");
        #endif
        return false;
    }

    #ifdef DEBUG
    // if message is for this device, or broadcast, print details:
    Serial.println();
    Serial.println("Received from: 0x" + String(sender, HEX));
    Serial.println("Sent to: 0x" + String(recipient, HEX));
    Serial.println("Command: " + String(command));
    Serial.println("Message length: " + String(incomingLength));
    Serial.println("Message: " + incoming);
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    Serial.println();
    #endif

    lora_packet.message = incoming;
    lora_packet.command = command;
    lora_packet.sender = sender;
    if ((lora_packet.sender == LORA_ROCKET) && CALCULATE_DISTANCE)
    {
        lora_packet.distance = get_distance(parse_str(lora_packet.message, LAT_POS), parse_str(lora_packet.message, LNG_POS));
    }
    return true;
}


void get_rocket_telemetry()
{
    unsigned long ts = millis();
    if (onReceive(LoRa.parsePacket()))
    {
        Serial.println(lora_packet.message);
    }
    if (CALCULATE_DISTANCE)
    {
        Serial.println("RSSI: " + String(LoRa.packetRssi()));
        Serial.println("Snr: " + String(LoRa.packetSnr()));
        Serial.println("Distance" + String(lora_packet.distance));
        beep(frequency);
    }
}

void beep(int ghz)
{
    tone(PIN_BUZZER, ghz, standard_delay);
    delay(standard_delay);
    pinMode(PIN_BUZZER, INPUT);
    delay(standard_delay);
}


// GPS converter

String parse_str(String string_to_parse, int position)
{
    char buf[1024] = myString.toCharArray(buf, string_to_parse.length());
    int pos1 = 0, pos2 = 0, counter = 0;
    for (int i = 0; i < 1024; i++)
    {
        if (buf[i] == ",")
        {
            if (counter + 1 == position)
            {
                if (!pos1)
                {
                    pos1 = i + 1;
                }
                else
                {
                    pos2 = i;
                    break;
                }
            }
            else
            {
                counter++;
            }
        }
    }
    return string_to_parse.substring(pos1, pos2);
}


int get_distance(String sender_lat, String sender_lng)
{
    double earth_radius_km = 6371;

    double s_lat = double(sender_lat), s_lng = sender_lng;
    double c_lat = double(String(gps.location.lat())), c_lng = double(String(gps.location.lng()));

    double d_lat = degrees_to_radians(c_lat-s_lat);
    double d_lng = degrees_to_radians(c_lng-s_lng);

    s_lat = degrees_to_radians(s_lat);
    c_lat = degrees_to_radians(c_lat);

    double a = sin(d_lat/2.0) * sin(d_lat/2.0) + sin(d_lng/2.0) * sin(d_lng/2.0) * cos(s_lat) * Math.cos(c_lat);
    double c = 2.0 * atan2(sqrt(a), sqrt(1-a));
    return earth_radius_km * c;
}


double degrees_to_radians(degrees)
{
    return degrees * M_PI / 180;
}
