/* ***************************************************************************
 * This sketch contains logic of rc-controlled ignitor.                      *
 *                                                                           *
 * Sketch uses Arduino Nano as main board.                                   *
 * All communication via LoRA module RF433, with LED, relay module, 2x18650  *
 *                                                                           *
 * Logic:                                                                    *
 *    1) Init LED, relay pin, LoRa                                           *
 *    2) Check that fuse is ok, blink otherwise                              *
 *    3) Wait until LoRa control command received                            *
 *    4) Set demanded state and report to sender                             *
 *    5) If state is ignite, activate relay (fuse), report before and after  *
 *       ignition                                                            *
 *    6) Repeat 4 until error, in case of error - blink                      *
 *                                                                           *
 * Sketch written by Iliya Vereshchagin 2021.                                *
 *****************************************************************************/

//#define DEBUG

#include <SPI.h>
#include <LoRa.h>


// LED
const int LED_PIN = 6;
const int LED_IGNITOR_ERROR = 500;
const int LED_LORA_ERROR = 2000;
const int LED_STATE_ERROR = 5000;


// Ignitor
const int IGNITOR_PIN = 4;
const int IGNITOR_REFERENCE_PIN = 5;
const int IGNITOR_CONTROL_PIN = 2;
const int IGNITOR_THRESHOLD = 200;  // 204 is 5V
const int IGNITE_DELAY = 5000;
const int AVG_MEAS_COUNT = 64; // in case if circuit is broken, average measurement will be less threshold,
                               // but some of them may be greater, to ensure circuit is broken, measure several times


// LoRa
const int LORA_POWER = 20;                // set TX power to maximum
const int LORA_RETRIES = 12;              // try to init LoRa several times before error
const int LORA_DELAY = 500;               // delay between retries
const int LORA_SEND_DELAY = 100;           // delay between send data
const int LORA_SEND_RETRIES = 5;          // how much packets will be sent
const byte LORA_LOCAL_ADDRESS = 0xBB;     // address of this device
const byte LORA_DESTINATION = 0xAA;       // destination to send to
const byte LORA_KEYWORD = 0x42;           // verification keyword
const int LORA_RECEIVE_DELAY = 500;       // stop activity if correct packet received


// State
int state = 0;
const int STATE_DEFAULT = 0;
const int STATE_CONTROL = 10;
const int STATE_ARMED = 24;
const int STATE_IGNITION = 66;
const int STATE_IGNITED = 100;
const int STATE_OFF = 120;
const int STATE_ERROR = 200;


void setup()
{
    Serial.begin(9600);
    init_LED();
    init_ignitor();
    init_LoRa();
}


void loop()
{
    onReceive(LoRa.parsePacket());
}

// Init functions

void init_ignitor()
{
    pinMode(IGNITOR_REFERENCE_PIN, OUTPUT);
    digitalWrite(IGNITOR_REFERENCE_PIN, HIGH);

    if (ignitor_consistency() < IGNITOR_THRESHOLD)
    {
        #ifdef DEBUG
        Serial.println("Ignitor init failed!");
        #endif
        blink_LED(LED_IGNITOR_ERROR);
    }
    else
    {
        pinMode(IGNITOR_PIN, OUTPUT);
        turn_on_LED();
        #ifdef DEBUG
        Serial.println("Ignitor init OK");
        #endif
    }
}


void init_LED()
{
    pinMode(LED_PIN, OUTPUT);
    #ifdef DEBUG
    Serial.println("Init LED OK");
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
        blink_LED(LED_LORA_ERROR);
    }

    LoRa.setTxPower(LORA_POWER);  // aplify TX power
    #ifdef DEBUG
    Serial.println("LoRa started!");
    #endif
}


// Ignitor functions

void ignite()
{
    #ifdef DEBUG
    Serial.println("Ignition!");
    #endif
    digitalWrite(IGNITOR_PIN, HIGH);
    delay(IGNITE_DELAY);
    digitalWrite(IGNITOR_PIN, LOW);
    #ifdef DEBUG
    Serial.println("Ignited...");
    #endif
}


byte ignitor_consistency()
{
    int average_voltage = 0;
    for (int i=0; i < AVG_MEAS_COUNT; i++)
    {
        average_voltage += byte(analogRead(IGNITOR_CONTROL_PIN)/5);
    }
    average_voltage /= AVG_MEAS_COUNT;
    #ifdef DEBUG
    Serial.print("Normalized voltage:\t");
    Serial.println(average_voltage);
    #endif
    return (byte)average_voltage;
}


// Led functions

void turn_on_LED()
{
    digitalWrite(LED_PIN, HIGH);
}


void turn_off_LED()
{
    digitalWrite(LED_PIN, LOW);
}

void blink_LED(int timeout)
{
    while(true)
    {
        turn_on_LED();
        delay(timeout);
        turn_off_LED();
        delay(timeout);
    }
}


// LoRa functions

void send_message(String outgoing, byte command)
{
    for (int i=0; i < LORA_SEND_RETRIES; i++)
    {
        LoRa.beginPacket();                   // start packet
        LoRa.write(LORA_DESTINATION);         // add destination address
        LoRa.write(LORA_LOCAL_ADDRESS);       // add sender address
        LoRa.write(LORA_KEYWORD);             // add message ID
        LoRa.write(command);                  // use command placeholder for telemetry
        LoRa.write(outgoing.length());        // add payload length
        LoRa.print(outgoing);                 // add payload
        LoRa.endPacket();                     // finish packet and send it
        delay(LORA_SEND_DELAY);
    }
}

void onReceive(int packetSize)
{
    if (packetSize == 0)
    {
        return;                           // if there's no packet, return
    }

    // read packet header bytes:
    byte recipient = LoRa.read();         // recipient address
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
        return;
    }

    // if the recipient isn't this device or broadcast,
    if (recipient != LORA_LOCAL_ADDRESS)
    {
        #ifdef DEBUG
        Serial.println("This message is not for me.");
        #endif
        return;
    }

    if (magic_word != LORA_KEYWORD)
    {
        #ifdef DEBUG
        Serial.println("Wrong magic word, aborted");
        #endif
        return;
    }

    #ifdef DEBUG
    // if message is for this device, or broadcast, print details:
    Serial.println("Received from: 0x" + String(sender, HEX));
    Serial.println("Sent to: 0x" + String(recipient, HEX));
    Serial.println("Message ID: " + String(command));
    Serial.println("Message length: " + String(incomingLength));
    Serial.println("Message: " + incoming);
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    Serial.println();
    #endif

    delay(LORA_RECEIVE_DELAY);  // force delay before act
    switch(command)
    {
        case STATE_CONTROL:
        set_control();
        break;
        case STATE_ARMED:
        set_armed();
        break;
        case STATE_IGNITION:
        set_ignite();
        break;
        case STATE_OFF:
        set_off();
        break;
        default:
        set_error(STATE_ERROR);
        break;
    };
    #ifdef DEBUG
    Serial.println("State " + String(command) + " set, return to listening");
    #endif
}

// Control commands

void set_error(byte state)
{
    String message = "Error on state " + String(state);
    send_message(message, state);
    blink_LED(LED_STATE_ERROR);
}

void set_control()
{
    if (state == STATE_DEFAULT)
    {
        byte ignitor_state = byte(ignitor_consistency()/40);
        String message = "Control V=" + String(int(ignitor_state));
        send_message(message, ignitor_state);
        state = STATE_CONTROL;
        #ifdef DEBUG
        Serial.println("Sending message: " + message + " | " + String(ignitor_state));
        #endif
    }
    else
    {
        set_error(state);
    }
}

void set_armed()
{
    if (state == STATE_CONTROL)
    {
        String message = "Ignitor armed!";
        state = STATE_ARMED;
        send_message(message, state);
        #ifdef DEBUG
        Serial.println("Sending message: " + message + " | " + String(state));
        #endif
    }
    else
    {
        set_error(state);
    }
}


void set_ignite()
{
    if (state == STATE_ARMED)
    {
        String message = "Ignition!";
        state = STATE_IGNITION;
        send_message(message, state);
        #ifdef DEBUG
        Serial.println("Sending message: " + message + " | " + String(state));
        #endif
        ignite();
        message = "Ignited...";
        state = STATE_IGNITED;
        send_message(message, state);
        #ifdef DEBUG
        Serial.println("Sending message: " + message + " | " + String(state));
        #endif
    }
    else
    {
        set_error(state);
    }
}


void set_off()
{
    if (state == STATE_IGNITED)
    {
        String message = "Ignitor turned off";
        state = STATE_OFF;
        send_message(message, state);
        #ifdef DEBUG
        Serial.println("Sending message: " + message + " | " + String(state));
        #endif
        turn_off_LED();
    }
    else
    {
        set_error(state);
    }
}
