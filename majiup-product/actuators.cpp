#include "actuators.h"

void actuateDevice(uint8_t gpio, uint8_t value) {
  digitalWrite(gpio, value);
}