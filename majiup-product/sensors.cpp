#include "Arduino.h"
#include "sensors.h"

volatile uint16_t pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
float flowRate = 0.0;
float totalLiters = 0.0;
unsigned long previousMillis = 0;

void setupSensor(){
  pinMode(SENSOR_PWR_PIN, OUTPUT);

  //Power on sensor
  digitalWrite(SENSOR_PWR_PIN, HIGH);
}

void pulseCounter() {
  unsigned long now = millis();
  if (now - lastPulseTime > 5) {  // Ignore pulses happening within 5ms
    pulseCount++;
    lastPulseTime = now;
  }
}

// Function to initialize the flow sensor
void setupFlowSensor() {
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);
}

// Function to get the current flow rate
float getFlowRate() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;

    flowRate = (pulseCount / (CALIBRATION_FACTOR));  // Adjust based on calibration

    totalLiters += (flowRate / 60.0f);

    Serial.print("Pulse Count: ");
    Serial.println(pulseCount);

    pulseCount = 0;
  }

  return flowRate;
}

// Function to get the total volume of water that has passed
float getTotalVolume() {
  return totalLiters;
}
