#include "TemperatureSensor.h"

#define TEMPERATURE_BUS 2
#define TEMPERATURE_RESOLUTION 9

TemperatureSensor tempSensor(TEMPERATURE_BUS, TEMPERATURE_RESOLUTION);

void setup() {
  Serial.begin(9600);
  tempSensor.Temp_voidBegin();

  Serial.print("Parasite power is: ");
  if (tempSensor.Temp_boolIsParasitePowered()) {
    Serial.println("ON");
  } else {
    Serial.println("OFF");
  }
}

void loop() {
  Serial.print("Requesting temperatures...");
  tempSensor.Temp_voidPrintTemperature();
  delay(1000);
}
