#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

#define FLOW_SENSOR_PIN 3
#define CALIBRATION_FACTOR  10
#define SENSOR_PWR_PIN 6

void setupSensor();
float getFlowRate();
float getTotalVolume();

#endif