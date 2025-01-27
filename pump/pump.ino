#include <WaziDev.h>
#include <xlpp.h>
#include <Base64.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

#define pumpPin 8

boolean pumpState = LOW;
boolean dataReceived = false;

// NwkSKey (Network Session Key) and Appkey (AppKey) are used for securing LoRaWAN transmissions.
// You need to copy them from/to your LoRaWAN server or gateway.
// You need to configure also the devAddr. DevAddr need to be different for each devices!!
// Copy'n'paste the DevAddr (Device Address): 22011D05
unsigned char devAddr[4] = {0x22, 0x01, 0x1D, 0x05};

// Copy'n'paste the key to your Wazigate: 23158D3BBC31E6AF670D195B5AED5525
unsigned char appSkey[16] = {0x23, 0x15, 0x8D, 0x3B, 0xBC, 0x31, 0xE6, 0xAF, 0x67, 0x0D, 0x19, 0x5B, 0x5A, 0xED, 0x55, 0x25};

// Copy'n'paste the key to your Wazigate: 23158D3BBC31E6AF670D195B5AED5525
unsigned char nwkSkey[16] = {0x23, 0x15, 0x8D, 0x3B, 0xBC, 0x31, 0xE6, 0xAF, 0x67, 0x0D, 0x19, 0x5B, 0x5A, 0xED, 0x55, 0x25};

WaziDev wazidev;


void blinkLed(int times, int durationMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(8, HIGH);
    delay(durationMs);
    digitalWrite(8, LOW);
    delay(durationMs);
  }
}

void actuatePump(boolean state){
    digitalWrite(pumpPin, state);
    storeState(state);
}

void storeState(boolean state){
  EEPROM.write(0, state);
}

boolean retrieveState(){
  boolean state = EEPROM.read(0);
  return state;
}

void setup() {

  Serial.begin(38400);
  wazidev.setupLoRaWAN(devAddr, appSkey, nwkSkey);
  
  pinMode(pumpPin, OUTPUT);
  // put your setup code here, to run once:

  // 3 blinks device setup is success
  blinkLed(3,200);

  pumpState = retrieveState();

  serialPrintf("STORED STATE: %d\n", pumpState);

  actuatePump(pumpState);
}

XLPP xlpp(120);

void loop() {

  // put your main code here, to run repeatedly:
  uint8_t e = wazidev.sendLoRaWAN(xlpp.buf, xlpp.len);

  // Receive LoRaWAN message (waiting for *5 seconds only).
  uint8_t offs = 0;
  
  e = wazidev.receiveLoRaWAN(xlpp.buf, &xlpp.offset, &xlpp.len, 5000);
  
  if (e != 0)
  {
    dataReceived = false;
    if (e == ERR_LORA_TIMEOUT){
//      serialPrintf("nothing received\n");
    }
    else
    {
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
  StaticJsonDocument<100> doc; // Adjust size as needed
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    serialPrintf("JSON parse failed: ");
    serialPrintf(error.c_str());
    actuatePump(LOW); // Set to default safe state if parsing fails
    return;
  }

  // Get the "state" value from JSON and control pump
  int state = doc["state"];
  if (state == 1) {
    actuatePump(HIGH);
  }  else if(state==0){
    actuatePump(LOW);
    }
    else {
      actuatePump(LOW);
  }
} else {
  serialPrintf("No payload received from Gateway");
}
  

  delay(50);

  Serial.println();
  Serial.println();
}
