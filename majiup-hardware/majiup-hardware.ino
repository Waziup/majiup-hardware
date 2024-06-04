/*
  JSN-SR04T-V3.0 Ultrasonic Sensor - Mode 0 Demo

  Mode 0 is default mode with no jumpers or resistors (emulates HC-SR04)

  https://dronebotworkshop.com

  Original source code: https://wiki.keyestudio.com/KS0429_keyestudio_TDS_Meter_V1.0#Test_Code
  Project details: https://RandomNerdTutorials.com/arduino-tds-water-quality-sensor/

*/

#include <WaziDev.h>
#include <xlpp.h>
#include <Base64.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

// Defining sonar sensor
#define TRIGPIN A2    //4
#define ECHOPIN A1   //3
#define TotalReads 5
#define powerSonarPin 6

/*---------*/

//Defining timing params
unsigned long previousSendTime = 0;
unsigned long sendInterval = 15000; // Time in milliseconds 1s = 1000ms
// NwkSKey (Network Session Key) and Appkey (AppKey) are used for securing LoRaWAN transmissions.
// You need to copy them from/to your LoRaWAN server or gateway.
// You need to configure also the devAddr. DevAddr need to be different for each devices!!
// Copy'n'paste the DevAddr (Device Address): 26018EEE
unsigned char devAddr[4] = {0x26, 0x01, 0x8E, 0xEE};

// Copy'n'paste the key to your Wazigate: 23158D3BBC31E6AF670D195B5AED5501
unsigned char appSkey[16] = {0x23, 0x15, 0x8D, 0x3B, 0xBC, 0x31, 0xE6, 0xAF, 0x67, 0x0D, 0x19, 0x5B, 0x5A, 0xED, 0x55, 0x01};

// Copy'n'paste the key to your Wazigate: 23158D3BBC31E6AF670D195B5AED5501
unsigned char nwkSkey[16] = {0x23, 0x15, 0x8D, 0x3B, 0xBC, 0x31, 0xE6, 0xAF, 0x67, 0x0D, 0x19, 0x5B, 0x5A, 0xED, 0x55, 0x01};

//Initializing wazidev class
WaziDev wazidev;

/*-------*/

XLPP xlpp(120);

// Filtering declarations...
const int threshold = 15; // Thresholding value
const int stabilityThreshold = 10; // Number of consecutive measurements within threshold
bool calibrating = true;
float previousValue = 0;
int stabilityCount = 0;
float lastStableValue = 0;

float getWaterLevel() {
  digitalWrite(powerSonarPin, HIGH);
  unsigned int sum = 0;
  // Floats to calculate distance
  float duration, distance;

  for (int i = 0; i != TotalReads; i++)
  {
    // Set the trigger pin LOW for 2uS
    digitalWrite(TRIGPIN, LOW);
    delayMicroseconds(2);

    // Set the trigger pin HIGH for 20us to send pulse
    digitalWrite(TRIGPIN, HIGH);
    delayMicroseconds(20);

    // Return the trigger pin to LOW
    digitalWrite(TRIGPIN, LOW);

    // Measure the width of the incoming pulse
    duration = pulseIn(ECHOPIN, HIGH);

    // Determine distance from duration
    // Use 343 metres per second as speed of sound
    // Divide by 1000 as we want millimeters

    distance = (duration / 2) * 0.343;
    sum += distance;
    delay(100);
  }
  digitalWrite(powerSonarPin, LOW);
  return sum / TotalReads;

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
  serialPrintf("LoRa receive ... ");
  xlpp.reset();
  uint8_t e = wazidev.sendLoRaWAN(xlpp.buf, xlpp.len);
  uint8_t offs = 0;
  long startSend = millis();
  uint8_t offset;
  uint8_t len;
  uint8_t buf[255];

  //e = wazidev.receiveLoRaWAN(xlpp.buf, &offset, &xlpp.len, 6000);
  e = wazidev.receiveLoRaWAN(xlpp.buf, &xlpp.offset, &xlpp.len, 10000);

  long endSend = millis();
  if (e != 0)
  {
    if (e == ERR_LORA_TIMEOUT) {
      serialPrintf("nothing received\n");
    }
    else
    {
      serialPrintf("Err %d\n", e);
    }
    delay(500);
    return;
  }
  serialPrintf("OK\n");
  serialPrintf("Payload: ");
  char payload[100];
  base64_decode(payload, xlpp.getBuffer(), xlpp.len);
  serialPrintf(payload);
  if (payload) {
  }
  serialPrintf("\n");
}

void setup() {
  // Set up serial monitor
  Serial.begin(38400);

  /*---------*/

  uint8_t errSetup = wazidev.setupLoRaWAN(devAddr, appSkey, nwkSkey);
  if (errSetup != 0)
  {
    serialPrintf("LoRaWAN Err %d\n", errSetup);
    delay(50);
    return;
  }

  else {
    serialPrintf("LoRaWAN setup successful\n");
  }

  serialPrintf("Starting Majiup...");

  /*--------*/

  // Set pinmodes for water level sensor connections
  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);


  pinMode(powerSonarPin, OUTPUT);
}

void loop() {

  xlpp.reset();  //reset the payload in every loop

  float level = getWaterLevel();
  
  Serial.print("Water Level: ");
  Serial.print(level);
  Serial.print(" mm");
  Serial.println();

  if (stabilityCount <= stabilityThreshold) {
    calibrating = true;
  }
  if (stabilityCount > stabilityThreshold) {
    calibrating = false;
  }

  // Measured value within threshold and calibrating (less than X values measured)
  if (abs(level - previousValue) <= threshold && calibrating) {
    Serial.print("Calibrating at ");
    serialPrintf("%d/%d", stabilityCount, stabilityThreshold);
    Serial.println();
    lastStableValue = previousValue;
    stabilityCount++;
  }

  if (abs(level - previousValue) > threshold && calibrating) {
    serialPrintf("In Calibration mode but sensor value out of threshold of %dmm.\nRestarting calibration", threshold);
    stabilityCount = 0;
  }

  // Measured value within threshold and not calibrating
  if (abs(level - lastStableValue) <= threshold && !calibrating) {

    lastStableValue = level;
    if (level > 0) {
      xlpp.addTemperature(0, level);
    }
  } else {
    if (!calibrating) {
      stabilityCount = 0;
      calibrating = true;

//      level = lastStableValue;
//      if (level > 0) {
//        xlpp.addTemperature(0,   level);
//      }
    }
  }

  previousValue = level;
  
  static unsigned long analogSampleTimepoint = millis();

//  Send data to gateway at given time intervals (sendInterval)
 unsigned long currentMillis = millis();
 if ((currentMillis - previousSendTime >= sendInterval) && !calibrating) {
    sendDataToGateway();
    previousSendTime = currentMillis;
  }

//  receiveLoRaData();

  // Delay before repeating measurement
  Serial.println("------------------------------------------------------------");
  delay(1000);
}
