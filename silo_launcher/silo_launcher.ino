/* ***************************************************************************
 * This sketch contains logic of self-maintained silo launcher.              *
 *                                                                           *
 * Sketch uses Arduino Nano as main board.                                   *
 * Sketch uses piezzo buzzer, servo, two relay modules to control ignitor,   *
 * solenoid valve. It's recommended to power board, servo and ignitor with   *
 * stable 2x18650. Solenoid valve powered by 12v moto battery.               *
 *                                                                           *
 * Logic:                                                                    *
 *    1) Init all modules                                                    *
 *    2) Beep after init is complete                                         *
 *    3) Wait a safety delay (i.e. 1 min)                                    *
 *    4) Beep several times as final countdown                               *
 *    5) Open silo cap                                                       *
 *    6) Ignite engine                                                       *
 *    7) Open valve to push out rocket from silo                             *
 *    8) Switch off ignition                                                 *
 *    9) Close valve                                                         *
 *   10) Close cap                                                           *
 *   11) Beep continuously                                                   *
 *                                                                           *
 * Sketch written by Iliya Vereshchagin 2021.                                *
 *****************************************************************************/

#include <Servo.h>

#define DEBUG

// Servo globals
Servo servo;
const int SERVO_PIN = 10;
const int SERVO_DELAY = 1000;

// Solenoid valve globals
const int VALVE_PIN = 7;

// Ignitor globals
const int IGNITOR_PIN = 5;
const long IGNITOR_DELAY = 2000;

// Buzzer globals
const int BUZZER_PIN = 3;
const int BUZZER_FREQUENCY = 3500;
const int BUZZER_DELAY = 500;
const int BUZZER_LAST_DELAY = 5000;
const int BUZZER_OK = 1;
const int BUZZER_START = 3;

// Sequence globals
const long LAUNCH_DELAY = 60000;    // 1 minute from power on to start
const int POST_LAUNCH_DELAY = 3000; // 3 seconds


void setup() 
{
    #ifdef DEBUG
    Serial.begin(9600);
    #endif
    init_valve();
    init_ignitor();
    init_servo();
    init_buzzer();
    beep_count(BUZZER_OK);
    // Force launch sequence right after setup
    launch_sequence();
}


/* Init functions */

void init_ignitor()
{
    pinMode(IGNITOR_PIN, OUTPUT);
    digitalWrite(IGNITOR_PIN, LOW);
    #ifdef DEBUG
    Serial.println("Ignitor initialized");
    #endif
}

void init_valve()
{
    pinMode(VALVE_PIN, OUTPUT);
    digitalWrite(VALVE_PIN, LOW);
    #ifdef DEBUG
    Serial.println("Solenoid valve initialized");
    #endif
}

void init_servo()
{
    pinMode(SERVO_PIN, OUTPUT);
    servo.attach(SERVO_PIN);
    close_cap();
    #ifdef DEBUG
    Serial.println("Servo init done");
    #endif
}

void init_buzzer()
{
    pinMode(BUZZER_PIN, INPUT);
    #ifdef DEBUG
    Serial.println("Buzzer pin set");
    #endif
}


/* Control functions */

void open_valve()
{
    #ifdef DEBUG
    Serial.println("Opening solenoid valve");
    #endif
    digitalWrite(VALVE_PIN, HIGH);
}

void close_valve()
{
    #ifdef DEBUG
    Serial.println("Closing solenoid valve");
    #endif
    digitalWrite(VALVE_PIN, LOW);
}

void ignite()
{
    #ifdef DEBUG
    Serial.println("Ignition!");
    #endif
    digitalWrite(IGNITOR_PIN, HIGH);
    delay(IGNITOR_DELAY);
}

void switch_off_ignitor()
{
    #ifdef DEBUG
    Serial.println("Ignitor switch-off");
    #endif
    digitalWrite(IGNITOR_PIN, LOW);
}

void open_cap()
{
    #ifdef DEBUG
    Serial.println("Opening silo cap");
    #endif
    servo.write(0);
    delay(SERVO_DELAY);
}

void close_cap()
{
    #ifdef DEBUG
    Serial.println("Closing silo cap");
    #endif
    servo.write(120);
    delay(SERVO_DELAY);
}

void beep(int ghz, int demanded_delay)
{
    tone(BUZZER_PIN, ghz, demanded_delay);
    delay(demanded_delay);
    pinMode(BUZZER_PIN, INPUT);
    delay(demanded_delay);
}

void beep_count(int repeats)
{
    for (int i = 0; i < repeats; i++)
    {
        beep(BUZZER_FREQUENCY, BUZZER_DELAY);
    }
}


/* Main logic */
void launch_sequence()
{
    #ifdef DEBUG
    Serial.println("Launch sequence initiated");
    #endif
    delay(LAUNCH_DELAY);
    #ifdef DEBUG
    Serial.println("Launch imminent");
    #endif
    beep_count(BUZZER_START);
    // Launch!
    open_cap();
    ignite();
    open_valve();
    #ifdef DEBUG
    Serial.println("Launch done, now tear-down");
    #endif
    // Teardown
    switch_off_ignitor();
    delay(POST_LAUNCH_DELAY);
    close_valve();
    close_cap();
    #ifdef DEBUG
    Serial.println("Teardown completed, calling for maintenance");
    #endif
}

void loop() 
{
    beep_count(BUZZER_OK);
    delay(BUZZER_LAST_DELAY);
}
