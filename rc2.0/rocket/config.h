#ifndef __LOCAL_CONFIG_H__
#define __LOCAL_CONFIG_H__

// Flight consts
const float altitude_threshold = 10.0;

// States
#define STATE_DEFAULT 0
#define STATE_ROCKET_CONTROL 128
#define STATE_ROCKET_ARMED 140
#define STATE_ROCKET_IN_FLIGHT 160
#define STATE_ROCKET_IN_FLIGHT_LANDING 180
#define STATE_ROCKET_LANDED 220
#define STATE_ROCKET_ERROR 240

// Ignitor const
const int IGNITOR_PIN = 14;
const int IGNITOR_CONTROL_PIN = 33;

#endif
