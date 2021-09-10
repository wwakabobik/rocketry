#include "lora_module.h"
#include "config.h"

// LoRa functions

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
        #ifdef ROCKET_DEBUG
        Serial.println("LoRa init failed.");
        #endif
    }

    LoRa.setTxPower(LORA_POWER);  // aplify TX power
    #ifdef ROCKET_DEBUG
    Serial.println("LoRa started!");
    #endif
}


void wait_for_LoRa_command()
{
    while(true)
    {
        if (onReceive(LoRa.parsePacket()) == STATE_ROCKET_ARMED)
        {
            break;
        }
    }
}

void send_message(String outgoing, byte command)
{
    for (int i=0; i < LORA_SEND_RETRIES; i++)
    {
        LoRa.beginPacket();                   // start packet
        LoRa.write(LORA_RECEIVER);            // add destination address
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
        return 0;                         // if there's no packet, return
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
        #ifdef ROCKET_DEBUG
        Serial.println("error: message length does not match length");
        #endif
        return -1;
    }

    // if the recipient isn't this device or broadcast,
    if (recipient != LORA_LOCAL_ADDRESS)
    {
        #ifdef ROCKET_DEBUG
        Serial.println("This message is not for me.");
        #endif
        return -1;
    }

    if (magic_word != LORA_KEYWORD)
    {
        #ifdef ROCKET_DEBUG
        Serial.println("Wrong magic word, aborted");
        #endif
        return -1;
    }

    #ifdef ROCKET_DEBUG
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
    #ifdef ROCKET_DEBUG
    Serial.println("State " + String(command) + " set, return to listening");
    #endif
    return command;
}
