#include "sensors.h"
#include "actuators.h"
#include "lorawan.h"

void setup() {
  Serial.begin(38400);
  setupLorawan();
}

void loop() {
  
  sendDataToGateway();
  receiveLoRaData();
}