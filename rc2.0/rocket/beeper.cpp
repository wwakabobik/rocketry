#include "beeper.h"

// Beeper logic

void init_beeper()
{
    pinMode(BEEPER_PIN, INPUT);
    #ifdef DEBUG
    Serial.println("Buzzer pin set");
    #endif
}

void beep(int ghz, int time)
{
    tone(BEEPER_PIN, ghz, demanded_delay);
    delay(demanded_delay);
    pinMode(BEEPER_PIN, INPUT);
    delay(demanded_delay);
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
