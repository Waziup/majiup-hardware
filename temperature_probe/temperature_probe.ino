#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS A4 // Pin connected to the DS18B20 data line

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(38400);
  sensors.begin();
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
}

void loop() {
  sensors.requestTemperatures();

  // Read temperature in Celsius
  float temperatureC = sensors.getTempCByIndex(0);

  // Read temperature in Fahrenheit
  float temperatureF = sensors.toFahrenheit(temperatureC);

  // Print temperatures
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.print(" °C / ");
  Serial.print(temperatureF);
  Serial.println(" °F");

  // Delay before taking the next measurement
  delay(1000);
}
