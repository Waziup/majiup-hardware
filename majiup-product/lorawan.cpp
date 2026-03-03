#include "lorawan.h"

XLPP xlpp(120);
WaziDev wazidev;

// Copy'n'paste the DevAddr (Device Address): 28018EEF
unsigned char devAddr[4] = { 0x28, 0x01, 0x8E, 0xEF };

// Copy'n'paste the key to your Wazigate: 23158D3BBC31E6AF670D195B5AED5501
unsigned char appSkey[16] = { 0x23, 0x15, 0x8D, 0x3B, 0xBC, 0x31, 0xE6, 0xAF, 0x67, 0x0D, 0x19, 0x5B, 0x5A, 0xED, 0x55, 0x01 };

// Copy'n'paste the key to your Wazigate: 23158D3BBC31E6AF670D195B5AED5501
unsigned char nwkSkey[16] = { 0x23, 0x15, 0x8D, 0x3B, 0xBC, 0x31, 0xE6, 0xAF, 0x67, 0x0D, 0x19, 0x5B, 0x5A, 0xED, 0x55, 0x01 };

void setupLorawan() {
  uint8_t errSetup = wazidev.setupLoRaWAN(devAddr, appSkey, nwkSkey);
  if (errSetup != 0) {
    serialPrintf("LoRaWAN Err %d\n", errSetup);
    delay(50);
    return;
  }

  else {
    serialPrintf("LoRaWAN setup successful\n");
  }
}

void sendDataToGateway() {
  serialPrintf("LoRaWAN sending ... ");
  uint8_t e = wazidev.sendLoRaWAN(xlpp.buf, xlpp.len);
  if (e != 0)
  {
    serialPrintf("Err %d\n", e);
    delay(50);
    return;
  }
  serialPrintf("OK\n");
}

void receiveLoRaData() {
  serialPrintf("LoRaWAN Receive ... ");

  bool dataReceived = false;

  uint8_t e = wazidev.sendLoRaWAN(xlpp.buf, xlpp.len);

  // Receive LoRaWAN message (waiting for *3 seconds only).
  uint8_t offs = 0;

  e = wazidev.receiveLoRaWAN(xlpp.buf, &xlpp.offset, &xlpp.len, 3000);

  if (e != 0) {
    dataReceived = false;
    if (e == ERR_LORA_TIMEOUT) {
      //      serialPrintf("nothing received\n");
    } else {
      serialPrintf("Err %d\n", e);
    }
    delay(500);
    return;
  } else {
    dataReceived = true;
  }

  serialPrintf("LoRaWAN OK\n");

  char payload[100];

  if (xlpp.len > 0 && dataReceived) {
    // Decode base64 to get the original payload
    base64_decode(payload, xlpp.getBuffer(), xlpp.len);

    serialPrintf("Payload: ");
    serialPrintf(payload);

    // Parse the JSON payload
    StaticJsonDocument<100> doc;  // Adjust size as needed
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      serialPrintf("JSON parse failed: ");
      // serialPrintf(error.c_str());
      // actuatePump(LOW);  // Set to default safe state if parsing fails
      return;
    }

    // Get the "state" value from JSON and control pump
    int state = doc["state"];

    printf(state);

  } else {
    serialPrintf("No payload received from Gateway");
  }

  delay(50);
  Serial.println();
}