#include "Arduino.h"
#include "actuators.h"
#include <WaziDev.h>

void setupActuators() {
  // set pin modes for actuation
  pinMode(ACTUATOR_PIN_1, OUTPUT);
  pinMode(ACTUATOR_PIN_2, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(RELAY_PWR_PIN, OUTPUT);

  // Activate relay power
  digitalWrite(RELAY_PWR_PIN, HIGH);

  //Switch off Relay Board
  digitalWrite(ACTUATOR_PIN_1, LOW);
  digitalWrite(ACTUATOR_PIN_2, LOW);
  digitalWrite(PUMP_PIN, LOW);

  serialPrintf("Actuator Setup Complete\n");
}

void actuateDevice(uint8_t gpio, uint8_t value) {
  digitalWrite(gpio, value);
}


void actuateValve(uint8_t value) {
  if (value == 1) {
    Serial.println("Turning ON...");
    digitalWrite(ACTUATOR_PIN_1, HIGH);
    digitalWrite(ACTUATOR_PIN_2, LOW);
  } else if (value == 0) {
    Serial.println("Turning OFF...");
    digitalWrite(ACTUATOR_PIN_1, LOW);
    digitalWrite(ACTUATOR_PIN_2, HIGH);
  } else {
    digitalWrite(ACTUATOR_PIN_1, LOW);
    digitalWrite(ACTUATOR_PIN_2, LOW);
  }
}