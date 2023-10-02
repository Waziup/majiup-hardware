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
#define TRIGPIN 4    //4
#define ECHOPIN 3   //3
#define TotalReads 5
#define powerSonarPin 6

// Defining TDS sensor
#define TdsSensorPin A1  //A1
#define VREF 5.0              // analog reference voltage(Volt) of the ADC
#define SCOUNT  1             // sum of sample point
#define powerTDSPin 7

// Defining temperature probe sensor
#define ONE_WIRE_BUS A4  //A4   // Pin connected to the DS18B20 data line
#define powerTempPin 5

// Defining pump
#define pumpPin A2
int pumpStatusAddress = 0;

/*---------*/

//Defining timing params
unsigned long previousSendTime = 0;
unsigned long sendInterval = 300000;
// NwkSKey (Network Session Key) and Appkey (AppKey) are used for securing LoRaWAN transmissions.
// You need to copy them from/to your LoRaWAN server or gateway.
// You need to configure also the devAddr. DevAddr need to be different for each devices!!
// Copy'n'paste the DevAddr (Device Address): 26018EAF
unsigned char devAddr[4] = {0x26, 0x01, 0x8E, 0xAF};

// Copy'n'paste the key to your Wazigate: 23158D3BBC31E6AF670D195B5AED5525
unsigned char appSkey[16] = {0x23, 0x15, 0x8D, 0x3B, 0xBC, 0x31, 0xE6, 0xAF, 0x67, 0x0D, 0x19, 0x5B, 0x5A, 0xED, 0x55, 0x25};

// Copy'n'paste the key to your Wazigate: 23158D3BBC31E6AF670D195B5AED5525
unsigned char nwkSkey[16] = {0x23, 0x15, 0x8D, 0x3B, 0xBC, 0x31, 0xE6, 0xAF, 0x67, 0x0D, 0x19, 0x5B, 0x5A, 0xED, 0x55, 0x25};

//Initializing wazidev class
WaziDev wazidev;

/*-------*/

XLPP xlpp(120);


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

// Filtering declarations...
//
const int threshold = 15; // Adjust this threshold as needed
const int stabilityThreshold = 30; // Adjust as needed
bool calibrating = true;
float previousValue = 0;
int stabilityCount = 0;
float lastStableValue = 0;
//

float averageVoltage = 0;
float tdsValue = 0;
//float temperature = 23;       // current temperature for compensation 9 (Read from sensor)

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
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
    if (strcmp(payload, "true") == 0) {
      Serial.println("Turn pump ON");
      digitalWrite(pumpPin, HIGH);
      writePumpStatus(1);
    } else if (strcmp(payload, "false") == 0) {
      Serial.println("Turn pump OFF");
      digitalWrite(pumpPin, LOW);
      writePumpStatus(0);
    } else {
      Serial.println("Unknown payload: " + String(payload));
    }
  }
  serialPrintf("\n");
}


// Function to read the pump status from EEPROM
bool readPumpStatus() {
  byte value = EEPROM.read(pumpStatusAddress);
  return (value == 1); // 1 represents ON, 0 represents OFF
}

// Function to write the pump status to EEPROM
void writePumpStatus(bool status) {
  byte value = status ? 1 : 0;
  EEPROM.write(pumpStatusAddress, value);
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

  /*--------*/

  // Set pinmodes for sensor connections
  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);

  pinMode(TdsSensorPin, INPUT);
  sensors.begin();

  pinMode(powerTDSPin, OUTPUT);
  pinMode(powerTempPin, OUTPUT);
  pinMode(powerSonarPin, OUTPUT);

  digitalWrite(powerTDSPin, HIGH);

  // Setup pump
  pinMode(pumpPin, OUTPUT);
  bool pumpStatus = readPumpStatus();
  digitalWrite(pumpPin, pumpStatus ? HIGH : LOW);
}

void loop() {

  xlpp.reset();

  float level = getWaterLevel();
  Serial.println(level);

  if (stabilityCount <= stabilityThreshold) {
    calibrating = true;
  }
  if (stabilityCount > stabilityThreshold) {
    calibrating = false;
  }

  // Measured value within threshold and calibrating (less than X values measured)
  if (abs(level - previousValue) <= threshold && calibrating) {
    Serial.println("Calibrating...");
    lastStableValue = previousValue;
    stabilityCount++;
    //    Serial.print("SV: ");
    //    Serial.println(lastStableValue);
    //    Serial.print("COUNT: ");
    //    Serial.println(stabilityCount);
  }

  if (abs(level - previousValue) > threshold && calibrating) {
    Serial.println("In Calib Mode but sensor val out of threshold");
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
      level = lastStableValue;
      if (level > 0) {
        xlpp.addTemperature(0, level);
      }
    }
  }

  previousValue = level;

  digitalWrite(powerTempPin, HIGH);
  //Request for temperature fromt the temperature probe
  sensors.requestTemperatures();

  // Read temperature in Celsius
  float temperatureC = sensors.getTempCByIndex(0);
  digitalWrite(powerTempPin, HIGH);


  Serial.println(temperatureC);
  
  static unsigned long analogSampleTimepoint = millis();

  if (millis() - analogSampleTimepoint > 40U) { //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;

      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationCoefficient = 1.0 + 0.02 * (temperatureC - 25.0);
      //temperature compensation
      float compensationVoltage = averageVoltage / compensationCoefficient;

      //convert voltage value to tds value
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      Serial.print("TDS Value:");
      Serial.print(tdsValue, 0);
      Serial.println("ppm");
    }
  }

  //check for zero values before sending the values. Each values is send independently
  if (temperatureC > 0) {
    xlpp.addTemperature(1, temperatureC);
  }
  if (tdsValue > 0) {
    xlpp.addTemperature(2, tdsValue);
  }

  // Send payload with LoRaWAN.

  unsigned long currentMillis = millis();
  if (currentMillis - previousSendTime >= sendInterval) {
    sendDataToGateway();
    previousSendTime = currentMillis;
  }

  receiveLoRaData();

  // Delay before repeating measurement
  Serial.println("------------------------------------------------------------");
  delay(1000);
}
