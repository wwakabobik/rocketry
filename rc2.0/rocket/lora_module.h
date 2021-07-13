#ifndef __LORA_MODULE_H__
#define __LORA_MODULE_H__

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


void init_LoRa();
void wait_for_LoRa_command();
void send_message(String outgoing, byte command);
int onReceive(int packetSize);

#endif