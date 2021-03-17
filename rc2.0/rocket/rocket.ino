/* ***************************************************************************
 * This sketch contains logic of rc-controlled flight computer.              *
 *                                                                           *
 * Sketch uses TTGO Meshtastic T-Beam board as base.                         *
 * All communication via LoRA module RF433, with connected BMP180,  MPU9050, *
 * relay(s) for chute deploy.                                                *
 *                                                                           *
 * Logic:                                                                    *
 *    1) Activate GPS, LoRa, BMP180, MPU9050, pins                           *
 *    2) If something wrong while init, start beep continuously              *
 *    3) Wait for GPS is connected                                           *
 *    4) Wait for control command, response accordingly                      *
 *    5) If command is "armed", start dump sensor data tp SPIFFS             *
 *    6) If jumper is out or altitude raise more 10 meters, start telemetry  *
 *    7) If altitude starts to falling more 5 meters after maximum altitude  *
 *       value, deploy chute                                                 *
 *    8) If chute timeout is reached but p. 7 is false, deploy chute         *
 *    9) Start beeping                                                       *
 *    10) If landed or flight timeout reached, stop telemetry, send GPS data *
 *                                                                           *
 * Sketch written by Iliya Vereshchagin 2021.                                *
 *****************************************************************************/

#define DEBUG

#include <SPI.h>              
#include <LoRa.h>


// LoRa
const int LORA_POWER = 20;                // set TX power to maximum
const int LORA_RETRIES = 12;              // try to init LoRa several times before error
const int LORA_DELAY = 500;               // delay between retries
const int LORA_SEND_DELAY = 100;          // delay between send data
const int LORA_SEND_RETRIES = 5;          // how much packets will be sent
const byte LORA_LOCAL_ADDRESS = 0xCC;     // address of this device
const byte LORA_RECEIVER = 0xAA;          // destination to send to
const byte LORA_KEYWORD = 0x42;           // verification keyword
const int LORA_RECEIVE_DELAY = 500;       // stop activity if correct packet received
const unsigned long LORA_TIMEOUT = (LORA_RECEIVE_DELAY + (LORA_SEND_DELAY * LORA_SEND_RETRIES * 10));


// State
const int STATE_DEFAULT = 0;
const int STATE_ROCKET_CONTROL = 128;
const int STATE_ROCKET_ARMED = 140;
const int STATE_ROCKET_IN_FLIGHT = 160;
const int STATE_ROCKET_IN_FLIGHT_LANDING = 180;
const int STATE_ROCKET_LANDED = 220;
const int STATE_ROCKET_ERROR = 240;


// Beeper consts
const int BEEP_FREQUENCY = 500;
const int BEEP_LONG = 500;
const int BEEP_DELAY = 200;
const int BEEP_CONT_DELAY = 5000;
const int BEEP_RESCUE_LONG = 1000;
const int BEEPER_PIN = ???;



void setup()
{
    #ifdef DEBUG
    Serial.begin(9600);
    #endif
    init_beeper();
    init_ignitor();
    init_LoRa();
    init_barometer();
    init_accelerometr();
    init_SPIFFS();
    init_GPS();
    wait_for_LoRa_command();
    wait_for_start();
}


void loop()
{
    // Seems we need to do several actions here
    // 1. get data from sensors
    // 2. Check for apogee / landing / timeouts
    // 3. Write to SPIFFS
    // 4. Send telemetry
}


// Init functions

void init_ignitor()
{
    // should I add consistency check here? Need to try...
}


void init_beeper()
{

}


void init_barometer()
{

}


void init_accelerometr()
{

}


void init_SPIFFS()
{

}


void init_GPS()
{

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
    return byte(analogRead(IGNITOR_CONTROL_PIN)/5);
}


// LoRa functions

void wait_for_LoRa_command()
{
    while(true)
    {
        if (onReceive(LoRa.parsePacket()) == STATE_ARMED)
        {
            break
        }
    }
}

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


int onReceive(int packetSize)
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
        return -1;
    }

    // if the recipient isn't this device or broadcast,
    if (recipient != LORA_LOCAL_ADDRESS) 
    {
        #ifdef DEBUG
        Serial.println("This message is not for me.");
        #endif
        return -1;
    }

    if (magic_word != LORA_KEYWORD)
    {
        #ifdef DEBUG
        Serial.println("Wrong magic word, aborted");
        #endif
        return -1;
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
        default:
        set_error(STATE_ERROR);
        break;
    };
    #ifdef DEBUG
    Serial.println("State " + String(command) + " set, return to listening");
    #endif
    return command;
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
        gps_data = get_gps_data();
        send_message(message, gps_data);
        state = STATE_CONTROL;
        #ifdef DEBUG
        Serial.println("Sending message: " + message + " | " + gps_data);
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
        String message = "Rocket armed!";
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


// Beeper logic

void beep(int ghz, int time)
{

}


void beep_state(int times):
{
    for(int i = 0; i < times; i++)
    {
        beep(BEEP_FREQUENCY, BEEP_LONG);
        delay(BEEP_DELAY);
    }
}

void beep_continuously(int state):
{
    while(true):
    {
        beep_state(state);
    }
}

void beep_rescue()
{
    beep(BEEP_FREQUENCY, BEEP_RESCUE_LONG));
    delay(BEEP_DELAY);
}


// Flight sequence logic

void wait_for_start()
{
    while(true)
    {
        if (((BMP180.altitude - land_altitude) > 10) or (digitalRead(JUMPER_PIN) != HIGH))
        {
            break;
        }
        else
        {
            sensor_data = get_sensors_data();
            store_spiffs_data(sensor_data);
        }
    }
}
