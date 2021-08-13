#include "beeper.h"

// Beeper logic

#ifdef    ARDUINO_ARCH_ESP32

#define SOUND_PWM_CHANNEL   0
#define SOUND_RESOLUTION    8 // 8 bit resolution
#define SOUND_ON            (1<<(SOUND_RESOLUTION-1)) // 50% duty cycle
#define SOUND_OFF           0                         // 0% duty cycle

void tone(int pin, int frequency, int duration)
{
  ledcSetup(SOUND_PWM_CHANNEL, frequency, SOUND_RESOLUTION);  // Set up PWM channel
  ledcAttachPin(pin, SOUND_PWM_CHANNEL);                      // Attach channel to pin
  ledcWrite(SOUND_PWM_CHANNEL, SOUND_ON);
  delay(duration);
  ledcWrite(SOUND_PWM_CHANNEL, SOUND_OFF);
}

#endif


void init_beeper()
{
    pinMode(BEEPER_PIN, INPUT);
    #ifdef ROCKET_DEBUG
    Serial.println("Buzzer pin set");
    #endif
}

void beep(int ghz, int demanded_delay)
{
    tone(BEEPER_PIN, ghz, demanded_delay);
    delay(demanded_delay);
    pinMode(BEEPER_PIN, INPUT);
    delay(demanded_delay);
}


void beep_state(int times)
{
    for(int i = 0; i < times; i++)
    {
        beep(BEEP_FREQUENCY, BEEP_LONG);
        delay(BEEP_DELAY);
    }
}

void beep_continuously(int state)
{
    while(true)
    {
        beep_state(state);
    }
}

void beep_rescue()
{
    beep(BEEP_FREQUENCY, BEEP_RESCUE_LONG);
    delay(BEEP_DELAY);
}
