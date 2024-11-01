/*
  JSN-SR04T-V3.0 Ultrasonic Sensor - Mode 0 Demo
  srt04-mode0.ino
  Uses JSN-SR04T-V3.0 Ultrasonic Sensor
  Displays on Serial Monitor
 
  Mode 0 is default mode with no jumpers or resistors (emulates HC-SR04)
 
  DroneBot Workshop 2021
  https://dronebotworkshop.com
*/

#include <WaziDev.h>
#include <xlpp.h>

#include <Base64.h>

 
// Define connections to sensor
#define TRIGPIN A2
#define ECHOPIN A1

#define TotalReads A1
#define powerSonarPin 6


 // Filtering declarations...
//
const int threshold = 15; // Adjust this threshold as neededX
const int stabilityThreshold = 10; // Adjust as needed
bool calibrating = true;
float previousValue = 0;
int stabilityCount = 0;
float lastStableValue = 0;


// Floats to calculate distance
//float/ duration, distance;
// Copy'n'paste the DevAddr (Device Address): 26018EEE
unsigned char devAddr[4] = {0x26, 0x01, 0x8E, 0xEE};

// Copy'n'paste the key to your Wazigate: 23158D3BBC31E6AF670D195B5AED5501
unsigned char appSkey[16] = {0x23, 0x15, 0x8D, 0x3B, 0xBC, 0x31, 0xE6, 0xAF, 0x67, 0x0D, 0x19, 0x5B, 0x5A, 0xED, 0x55, 0x01};

// Copy'n'paste the key to your Wazigate: 23158D3BBC31E6AF670D195B5AED5501
unsigned char nwkSkey[16] = {0x23, 0x15, 0x8D, 0x3B, 0xBC, 0x31, 0xE6, 0xAF, 0x67, 0x0D, 0x19, 0x5B, 0x5A, 0xED, 0x55, 0x01};

//Initializing wazidev class
WaziDev wazidev;

XLPP xlpp(120);


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
//  digitalWrite(powerSonarPin, LOW);
  return sum / TotalReads;

}



void setup() {

  Serial.begin(38400);

  /*--------*/
  // Set up serial monitor
  Serial.begin(38400);
 
  // Set pinmodes for sensor connections
  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
}
 
void loop() {

  float level = getWaterLevel();
  Serial.print("Water Level: ");
  Serial.print(level);
  Serial.print(" mm");
  Serial.println();
 
 // Delay before repeating measurement
  delay(100);
}
