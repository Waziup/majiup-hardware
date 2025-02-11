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
#include <LowPower.h>

// Defining sonar sensor
#define TRIGPIN A2    //4
#define ECHOPIN A1   //3
#define TotalReads 1
#define powerSonarPin 6

/*---------*/


#include <Vcc.h>

//MORE DECIMAL PLACES LEAD TO MUCH ACCURATE BATTERY READING(thus for VccMax and VccCorrection)
const float VccMin   = 0.0;           // Minimum expected Vcc level, in Volts.
const float VccMax   = 3.319;         // Maximum expected Vcc level, in Volts(Use a multimeter to measure the maximum output voltage of your regulator. Thus when you connect a fully charged battery to it).
const float VccCorrection = 0.9825;  // Tweak this value until the output of "v" on line 38 is the same as what you measured with your multimeter above for VccMax

const float lowBat = 2.9; //Indicate the value your battery manufacturer states as absolutely dead battery
const float fullBat = 4.8; //Indicate the value your fully charged battery will be at

static unsigned long lastSendTime = 0;  // Tracks the last time xlpp.addTemperature was executed
unsigned long currentMillis = millis(); // Get the current time

float percentage = 0.0;
Vcc vcc(VccCorrection);

//BATTERY IS CONNECTED TO A0
int batt_pin = A0;


//Defining timing params
unsigned long previousSendTime = 0;
unsigned long sendInterval = 300000; // Time in milliseconds 1s = 1000ms
// NwkSKey (Network Session Key) and Appkey (AppKey) are used for securing LoRaWAN transmissions.
// You need to copy them from/to your LoRaWAN server or gateway.
// You need to configure also the devAddr. DevAddr need to be different for each devices!!
// Copy'n'paste the DevAddr (Device Address): 28018EEF
unsigned char devAddr[4] = {0x28, 0x01, 0x8E, 0xEF};

// Copy'n'paste the key to your Wazigate: 23158D3BBC31E6AF670D195B5AED5501
unsigned char appSkey[16] = {0x23, 0x15, 0x8D, 0x3B, 0xBC, 0x31, 0xE6, 0xAF, 0x67, 0x0D, 0x19, 0x5B, 0x5A, 0xED, 0x55, 0x01};

// Copy'n'paste the key to your Wazigate: 23158D3BBC31E6AF670D195B5AED5501
unsigned char nwkSkey[16] = {0x23, 0x15, 0x8D, 0x3B, 0xBC, 0x31, 0xE6, 0xAF, 0x67, 0x0D, 0x19, 0x5B, 0x5A, 0xED, 0x55, 0x01};

//Initializing wazidev class
WaziDev wazidev;

/*-------*/

XLPP xlpp(120);

// Filtering declarations...
//const int threshold = 150; // Thresholding value
const int threshold = 200; // Thresholding value

const int stabilityThreshold = 5; // Number of consecutive measurements within threshold
bool calibrating = true;
float previousValue = 0;
int stabilityCount = 0;
float lastStableValue = 0;

float getWaterLevel() {
  // Power up the sonar sensor
  digitalWrite(powerSonarPin, HIGH);
  delay(100); // Stabilize the sensor (100ms is ideal for JSN-SRT04)

  float sum = 0.0;
  float duration, distance;
  int validReadings = 0;

  for (int i = 0; i < TotalReads; i++) {
    // Ensure the trigger pin is LOW before sending a pulse
    digitalWrite(TRIGPIN, LOW);
    delayMicroseconds(2);

    // Send a 10-microsecond pulse on the trigger pin
    digitalWrite(TRIGPIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGPIN, LOW);

    // Measure the width of the incoming pulse (timeout set to 35ms for ~6m range)
    duration = pulseIn(ECHOPIN, HIGH, 35000);

    // Calculate the distance (speed of sound = 343 m/s)
    distance = (duration / 2.0) * 0.343; // Distance in millimeters

    // Filter out invalid readings (below min or above max range)
    if (distance > 250 && distance < 6000) { // JSN-SRT04 range: 250mm to 6000mm
      sum += distance;
      validReadings++;
    }

    delay(60); // Short delay between readings
  }

  // Power down the sonar sensor
  digitalWrite(powerSonarPin, LOW);

  // Return the average distance or -1 if no valid readings
  return (validReadings > 0) ? (sum / validReadings) : 0;
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

float getBattVoltage() {  
  int j;
  float v = 0;
  for (j = 0; j < 100; j++) {
    v += vcc.Read_Volts();
    delay(5);
  }

  //FIND AVERAGE OF 100 VCC VOLTAGE SAMPLES
  v = v / j;

  //PRINTING AVERAGE OF 100 VCC VOLTAGE SAMPLES TO 3 DECIMAL PLACES
  Serial.println();
  Serial.print("VCC: ");
  Serial.println(v, 3);

  
  //COLLECT 100 BATTERY VOLTAGE SAMPLES
  int i;
  float batt_volt = 0;
  for (i = 0; i < 100; i++) {
    //USING THE CURRENT VCC VOLTAGE AS REFERENCE TO CALCULATE BATTERY VOLTAGE
    batt_volt += ((analogRead(batt_pin) * (v / 1023.0)) * 2);
    delay(5);
  }

  //FIND AVERAGE OF 100 BATTERY VOLTAGE SAMPLES
  batt_volt = batt_volt / i;

  //CONVERTING BATTERY VOLTAGE TO PERCENTAGE ASSUMING 4.19V AS THE FULL CHARGE VALUE
  float percentage = ((batt_volt - lowBat )/ (fullBat - lowBat)) * 100;
  return percentage;
}

bool getCharging() {
  return true;
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

  // initialize a timer that checks when xlpp.addTemperature is executed
  // if the the addtemperature was executed more than 2 minutes ago, execute xlpp.addTemperature(0,level)
  // When the addtemperature gets executed reset the timer
  // Its a forceful(bruteforce) way to send the current level
 
   
  if ((abs(level - lastStableValue) <= threshold || (currentMillis - lastSendTime >= 120000)) && !calibrating) {

    lastStableValue = level;
    
    if (level > 0) {
      xlpp.addTemperature(0, level);
      lastSendTime = currentMillis; // Reset the timer
    }
  } else {
    if (!calibrating) {
      stabilityCount = 0;
      calibrating = true;
    }
  }

  previousValue = level;
  
  static unsigned long analogSampleTimepoint = millis();

  
  float percentage = getBattVoltage();
  xlpp.addTemperature(1, percentage);

//  Send data to gateway at given time intervals (sendInterval)
    
    if (!calibrating){
      sendDataToGateway();
    }

  // Delay before repeating measurement
  Serial.println("------------------------------------------------------------");
      
  float  delayTime = (percentage > 50.00) ? 7 : 7;
  if (!calibrating){
    for (int i = 0; i < delayTime; i++) {
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    }
  }
  
}
