#include "sensors.h"
#include "actuators.h"
#include "lorawan.h"

#define BUILT_IN 8

void setup() {
  Serial.begin(38400);
  Serial.println("Initializing Majiup...");
  setupLorawan();
  setupActuators();
}

void loop() {
  // actuateValve(1);
  // delay(8000);

  // actuateValve(0);
  // delay(8000);

  float flowRate = getFlowRate();

  Serial.println(flowRate);
  delay(1000);

  
  // sendDataToGateway();
  // receiveLoRaData();
}
