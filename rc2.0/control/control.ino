/* ***************************************************************************
 * This sketch contains logic of multi-functional tranceiver                 *
 *                                                                           *
 * Sketch uses Arduino Nano as main board.                                   *
 * All communication via LoRA module RF433 support, indication on three LEDs *
 * and OLED display. Control via two switches and engine start button.       *
 *                                                                           *
 * Logic:                                                                    *
 *    1) Init LED, button and Wi-fi (RF433, LoRA)                            *
 *    2) Wait for switches triggered                                         *
 *    3) Get control valued from ignitor/rocket                              *
 *    4) Set "armed" state for ignitor/rocket                                *
 *    5) In case of error, stop init and blink with related LED              *
 *    6) Only when ignitor (optionally rocket) init OK, wait for engine      *
 *       start button pressed                                                *
 *    7) Receive rocket telemetry                                            *
 *    8) Go to p. 7                                                          *
 *                                                                           *
 * Sketch written by Iliya Vereshchagin 2021.                                *
 *****************************************************************************/

//#define DEBUG
//#define OLED_OUT
//#define PRINT_TELEMETRY

#include <SPI.h>
#include <LoRa.h>
#include <Arduino.h>
#ifdef OLED_OUT
#include <U8g2lib.h>
#endif

// LED
const int LED_POWER_PIN = 8;
const int LED_IGNITOR_PIN = 7;
const int LED_ROCKET_PIN = 6;
const int LED_BLINK_DELAY = 500;

// Buttons
const int BUTTON_IGNITOR_PIN = 5;
const int BUTTON_ROCKET_PIN = 4;
const int BUTTON_ENGINE_PIN = 3;
const int BUTTON_DELAY = 500;

// LoRa
const int LORA_POWER = 20;                // set TX power to maximum
const int LORA_RETRIES = 12;              // try to init LoRa several times before error
const int LORA_DELAY = 500;               // delay between retries
const int LORA_SEND_DELAY = 100;          // delay between send data
const int LORA_SEND_RETRIES = 5;          // how much packets will be sent
const byte LORA_LOCAL_ADDRESS = 0xAA;     // address of this device
const byte LORA_IGNITOR = 0xBB;           // destination to send to
const byte LORA_ROCKET = 0xCC;            // destination to send to
const byte LORA_KEYWORD = 0x42;           // verification keyword
const int LORA_RECEIVE_DELAY = 500;       // stop activity if correct packet received
const unsigned long LORA_TIMEOUT = (LORA_RECEIVE_DELAY + (LORA_SEND_DELAY * LORA_SEND_RETRIES * 10));

// State
const int STATE_IGNITION_CONTROL = 10;
const int STATE_IGNITION_ARMED = 24;
const int STATE_IGNITION_IGNITION = 66;
const int STATE_IGNITION_IGNITED = 100;
const int STATE_IGNITION_OFF = 120;
const int STATE_ROCKET_CONTROL = 128;
const int STATE_ROCKET_ARMED = 140;
const int STATE_ROCKET_IN_FLIGHT = 160;
const int STATE_ROCKET_LANDED = 220;

// Ignitor settings
const byte MIN_VOLTAGE = 4;
const int IGNITION_DELAY = 5000;

struct LoRa_packet{
  String message;
  byte command;
  byte sender;
} lora_packet;


// Main state
String rocket_gps = "";
const unsigned long telemetry_actual = 10 * 60 * 1000;  // 10 mins

#ifdef OLED_OUT
// OLED
U8G2_SH1106_128X32_VISIONOX_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
const int OLED_FIRST_LINE = 10;
const int OLED_SECOND_LINE = 25;
String first_line = "";
String second_line = "";
#endif

// Pass through rocket init any ignite
bool IGNORE_ROCKET = true;  // ignore rocket init for "engine start" check


// Main funtions

void setup()
{
    #ifdef DEBUG
    Serial.begin(9600);
    #endif
    init_LEDs();
    init_buttons();
    init_LoRa();
    turn_on_LED(LED_POWER_PIN);
    wait_for_button_pressed();
}


void loop()
{
    get_rocket_telemetry();
}


// Init functions

void init_LEDs()
{
    pinMode(LED_POWER_PIN, OUTPUT);
    pinMode(LED_IGNITOR_PIN, OUTPUT);
    pinMode(LED_ROCKET_PIN, OUTPUT);

    turn_off_LED(LED_POWER_PIN);
    turn_off_LED(LED_IGNITOR_PIN);
    turn_off_LED(LED_ROCKET_PIN);

    #ifdef DEBUG
    Serial.println("Init LED OK");
    #endif
}


void init_buttons()
{
    pinMode(BUTTON_IGNITOR_PIN, INPUT);
    pinMode(BUTTON_ROCKET_PIN, INPUT);
    pinMode(BUTTON_ENGINE_PIN, INPUT_PULLUP);

    #ifdef DEBUG
    Serial.println("Init buttons OK");
    #endif
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


// Led functions

void turn_on_LED(int pin)
{
    digitalWrite(pin, HIGH);
}


void turn_off_LED(int pin)
{
    digitalWrite(pin, LOW);
}


void blink_LED(int pin)
{
    while(true)
    {
        turn_on_LED(pin);
        delay(LED_BLINK_DELAY);
        turn_off_LED(pin);
        delay(LED_BLINK_DELAY);
    }
}


// LoRa functions

void send_message(String outgoing, byte command, byte destination)
{
    for (int i=0; i < LORA_SEND_RETRIES; i++)
    {
        LoRa.beginPacket();                   // start packet
        LoRa.write(destination);              // add destination address
        LoRa.write(LORA_LOCAL_ADDRESS);       // add sender address
        LoRa.write(LORA_KEYWORD);             // add message ID
        LoRa.write(command);                  // use command placeholder for telemetry
        LoRa.write(outgoing.length());        // add payload length
        LoRa.print(outgoing);                 // add payload
        LoRa.endPacket();                     // finish packet and send it
        delay(LORA_SEND_DELAY);
        #ifdef DEBUG
        Serial.println("LoRa message send (retry " + String(i) + ") with command: " + String(command));
        #endif
    }
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

    // if the recipient isn't this device or broadcast,
    if (recipient != LORA_LOCAL_ADDRESS)
    {
        #ifdef DEBUG
        Serial.println("This message is not for me.");
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

    delay(LORA_RECEIVE_DELAY);  // force delay before act
    lora_packet.message = incoming;
    lora_packet.command = command;
    lora_packet.sender = sender;
    return true;
}


// Control commands

bool init_ignitor()
{
    #ifdef DEBUG
    Serial.println("Ignition setup sequence initiated");
    #endif
    String message = "Command " + String(STATE_IGNITION_CONTROL);
    send_message("", STATE_IGNITION_CONTROL, LORA_IGNITOR);
    get_data_from_ignitor(LORA_TIMEOUT);
    #ifdef DEBUG
    Serial.println("Ignitor voltage is " + String(lora_packet.command));
    #endif
    if (lora_packet.command > MIN_VOLTAGE)
    {
        #ifdef DEBUG
        Serial.println("Go ignitor to armed state");
        #endif
        message = "Command " + String(STATE_IGNITION_ARMED);
        send_message("C", STATE_IGNITION_ARMED, LORA_IGNITOR);
    }
    else
    {
        #ifdef OLED_OUT
        first_line = "Fuse broken, " + String(lora_packet.command) + "V";
        print_OLED();
        #endif
        blink_LED(LED_IGNITOR_PIN);
    }
    get_data_from_ignitor(LORA_TIMEOUT);
    #ifdef DEBUG
    Serial.println("Ignitor state is " + String(lora_packet.command));
    #endif
    if (lora_packet.command == STATE_IGNITION_ARMED)
    {
        #ifdef DEBUG
        Serial.println("Ignitor in armed state");
        #endif
        turn_on_LED(LED_IGNITOR_PIN);
        return true;
    }
    else
    {
        #ifdef OLED_OUT
        first_line = "Ignitor broken!";
        print_OLED();
        #endif
        blink_LED(LED_IGNITOR_PIN);
        return false;
    }

}


bool ignite()
{
    send_message("", STATE_IGNITION_IGNITION, LORA_IGNITOR);
    get_data_from_ignitor(LORA_TIMEOUT);
    if (lora_packet.command != STATE_IGNITION_IGNITION)
    {
        #ifdef OLED_OUT
        first_line = "Ignitor broken!";
        print_OLED();
        #endif
        blink_LED(LED_IGNITOR_PIN);
    }
    get_data_from_ignitor(LORA_TIMEOUT + IGNITION_DELAY);
    if (lora_packet.command != STATE_IGNITION_IGNITED)
    {
        #ifdef OLED_OUT
        first_line = "Ignitor broken!";
        print_OLED();
        #endif
        blink_LED(LED_IGNITOR_PIN);
    }
    send_message("", STATE_IGNITION_OFF, LORA_IGNITOR);
    get_data_from_ignitor(LORA_TIMEOUT);
    if (lora_packet.command != STATE_IGNITION_OFF)
    {
        #ifdef OLED_OUT
        first_line = "Ignitor broken!";
        print_OLED();
        #endif
        blink_LED(LED_IGNITOR_PIN);
    }
    else
    {
        turn_off_LED(LED_IGNITOR_PIN);
    }
    return true;
}


bool init_rocket()
{
    while(digitalRead(BUTTON_ROCKET_PIN) != HIGH)
    {
        delay(BUTTON_DELAY);
    }
    send_message("", STATE_ROCKET_CONTROL, LORA_ROCKET);
    get_data_from_rocket();
    if (lora_packet.command == STATE_ROCKET_CONTROL)
    {
        rocket_gps = lora_packet.message;
        #ifdef OLED_OUT
        second_line = rocket_gps;
        print_OLED();
        #endif
        send_message("", STATE_ROCKET_ARMED, LORA_ROCKET);
    }
    else
    {
        #ifdef OLED_OUT
        second_line = "Rocket broken!";
        print_OLED();
        #endif
        blink_LED(LED_ROCKET_PIN);
    }
    get_data_from_rocket();
    if (lora_packet.command == STATE_ROCKET_ARMED)
    {
        turn_on_LED(LED_ROCKET_PIN);
    }
    else
    {
        #ifdef OLED_OUT
        second_line = "Rocket broken!";
        print_OLED();
        #endif
        blink_LED(LED_ROCKET_PIN);
    }
    return true;
}


void wait_for_button_pressed()
{
    #ifdef DEBUG
    Serial.println("Wait for any button pressed");
    #endif
    bool ignitor_ready = false;
    bool rocket_ready = false;
    while (true)
    {
        if ((digitalRead(BUTTON_ROCKET_PIN) == HIGH) && (rocket_ready == false))
        {
            rocket_ready = init_rocket();
        }

        if((digitalRead(BUTTON_IGNITOR_PIN) == HIGH) && (ignitor_ready == false))
        {
            ignitor_ready = init_ignitor();
        }

        if ((digitalRead(BUTTON_ENGINE_PIN) == LOW) && (ignitor_ready == true) && ((rocket_ready == true) or (IGNORE_ROCKET == true)))
        {
            ignite();
            break;
        }
        delay(BUTTON_DELAY);
    }
}

// Update LoRa data

void get_data_from_ignitor(int timeout)
{
    #ifdef DEBUG
    Serial.println("Listening ignitor echo");
    #endif
    unsigned long ts = millis();
    while(onReceive(LoRa.parsePacket()) != true)
    {
        if ((millis()-ts) > LORA_TIMEOUT*10)
        {
            #ifdef DEBUG
            Serial.println("timeout");
            #endif
            #ifdef OLED_OUT
            first_line = "I: Poor connection!";
            print_OLED();
            #endif
            blink_LED(LED_IGNITOR_PIN);
        }
    }
    #ifdef OLED_OUT
    first_line = lora_packet.message;
    print_OLED();
    #endif
}


void get_data_from_rocket()
{
    #ifdef DEBUG
    Serial.println("Listening rocket echo");
    #endif
    unsigned long ts = millis();
    while(onReceive(LoRa.parsePacket()) != true and lora_packet.sender != LORA_ROCKET)
    {
        // error
        if ((millis()-ts) > LORA_TIMEOUT)
        {
            #ifdef DEBUG
            Serial.println("timeout");
            #endif
            #ifdef OLED_OUT
            second_line = "R: poor connection!";
            print_OLED();
            #endif
            blink_LED(LED_ROCKET_PIN);
        }
    }
    #ifdef OLED_OUT
    second_line = lora_packet.message;
    print_OLED();
    #endif
}


void get_rocket_telemetry()
{
    unsigned long ts = millis();
    bool success = false;
    if (onReceive(LoRa.parsePacket()))
    {
        if (lora_packet.sender == LORA_ROCKET)
        {
            #ifdef PRINT_TELEMETRY
            Serial.println(lora_packet.message);
            #endif
            #ifdef OLED_OUT
            first_line = "Start: " + rocket_gps;
            second_line = lora_packet.message;
            print_OLED();
            #endif
        }
    }
}

#ifdef OLED_OUT
void print_OLED()
{
    char first_l[first_line.length()];
    first_line.toCharArray(first_l, second_line.length());
    char second_l[second_line.length()];
    second_line.toCharArray(second_l, second_line.length());
    u8g2.begin();
    u8g2.clearBuffer();                          // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr);          // choose a suitable font
    u8g2.drawStr(0, OLED_FIRST_LINE, first_l);   // write something to the internal memory
    u8g2.drawStr(0, OLED_SECOND_LINE, second_l); // write something to the internal memory
    u8g2.sendBuffer();                           // transfer internal memory to the display
}
#endif
