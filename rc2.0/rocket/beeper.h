#ifndef __BEEPER_H__
#define __BEEPER_H__

#include <Arduino.h>

// Beeper consts
const int BEEP_FREQUENCY = 500;
const int BEEP_LONG = 500;
const int BEEP_DELAY = 200;
const int BEEP_CONT_DELAY = 5000;
const int BEEP_RESCUE_LONG = 1000;
const int BEEPER_PIN = 13;

void init_beeper();
void beep(int ghz, int time);
void beep_state(int times);
void beep_continuously(int state);
void beep_rescue();

#endif
