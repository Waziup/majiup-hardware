#ifndef LORAWAN_H
#define LORAWAN_H

#include <xlpp.h>
#include <WaziDev.h>
#include <Base64.h>
#include <ArduinoJson.h>

extern WaziDev wazidev;
extern XLPP xlpp;

void setupLorawan();
void sendDataToGateway();
void receiveLoRaData();

#endif