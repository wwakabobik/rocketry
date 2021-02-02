/* ***************************************************************************
 * This sketch contains obtaining scale calibration factor for HX711         *
 *                                                                           *
 * Sketch uses Arduino UNO controller,  HX711 module, 20kg tensiometer.      *
 *                                                                           *
 * Third-party libraries:                                                    *
 *    - https://github.com/bogde/HX711.git                                   *
 *                                                                           *
 * Logic:                                                                    *
 *    1) Measure weight of standard                                          *
 *    2) Calculate and display proposed calibration factor for scale         *
 *                                                                           *
 * Sketch written by Iliya Vereshchagin 2018.                                *
 *****************************************************************************/

#include <HX711.h>

HX711 scale;
unsigned int weight_of_standard = 584;  // set your own standard ethalon

void setup()
{
    Serial.begin(9600);
    scale.begin(A1, A0);
    Serial.println("Before tare: ");
    Serial.println(scale.get_units(10));
    scale.tare();
    Serial.println("After tare: ");
    Serial.println(scale.get_units(10));
    Serial.println("\n\nPut standart weight on scale...\n");
    delay(10000);
    Serial.println("scale factor: ");
    Serial.println(scale.get_units(10)/weight_of_standard);
    float calibration_factor=scale.get_units(10)/weight_of_standard;
    Serial.println(String(calibration_factor));
    scale.set_scale(calibration_factor);
    delay(10000);
}

void loop()
{
    correction();
}

void correction() 
{
    float calibration_factor=scale.get_units(10)/weight_of_standard;
    Serial.println(String(calibration_factor));
    scale.set_scale(calibration_factor);
    delay(1000);
}
