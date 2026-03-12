#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <Arduino.h>

#define ACTUATOR_PIN_1 A1
#define ACTUATOR_PIN_2 A2
#define RELAY_PWR_PIN  7
#define PUMP_PIN 5

void actuateDevice(uint8_t gpio, uint8_t value);
void actuateValve(uint8_t value);
void setupActuators();

#endif