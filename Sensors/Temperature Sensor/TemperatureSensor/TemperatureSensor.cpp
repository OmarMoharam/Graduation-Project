// TemperatureSensor.cpp
#include "TemperatureSensor.h"

TemperatureSensor::TemperatureSensor(uint8_t Copy_uint8tPin, uint8_t Copy_uint8tResolution)
  : _pin(Copy_uint8tPin), _resolution(Copy_uint8tResolution), _oneWire(Copy_uint8tPin), _tempSensor(&_oneWire) {}

void TemperatureSensor::Temp_voidBegin() {
  Serial.begin(9600);
  _tempSensor.begin();
  //save address of index 0 to _tempAddress
  if (!_tempSensor.getAddress(_tempAddress, 0)) {
    Serial.println("Unable to find address for Device 0");
  } else {
    Serial.print("Device 0 Address: ");
    Temp_voidPrintAddress();
    Serial.println();
  }
  //set resolution for sensor
  _tempSensor.setResolution(_tempAddress, _resolution);
  Serial.print("Device 0 Resolution: ");
  Serial.println(_tempSensor.getResolution(_tempAddress));
}

void TemperatureSensor::Temp_voidSetResolution() {
  _tempSensor.setResolution(_tempAddress, _resolution);
}

bool TemperatureSensor::Temp_boolIsParasitePowered() {
  return _tempSensor.isParasitePowerMode();
}

void TemperatureSensor::Temp_voidPrintTemperature() {
  float Local_floatTempC = Temp_floatGetTemperatureC();
  // DEVICE_DISCONNECTED_C is -127 to check if the sensor is connected or not
  if (Local_floatTempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.print(Local_floatTempC);
  Serial.print(" Temp F: ");
  Serial.println(Temp_floatGetTemperatureF());
}

float TemperatureSensor::Temp_floatGetTemperatureC() {
  _tempSensor.requestTemperatures();
  return _tempSensor.getTempC(_tempAddress);
}

float TemperatureSensor::Temp_floatGetTemperatureF() {
  return DallasTemperature::toFahrenheit(Temp_floatGetTemperatureC());
}

void TemperatureSensor::Temp_voidPrintAddress() {
  for (uint8_t Local_uint8tCounter = 0; Local_uint8tCounter < 8; Local_uint8tCounter++) {
    if (_tempAddress[Local_uint8tCounter] < 16) Serial.print("0");
    Serial.print(_tempAddress[Local_uint8tCounter], HEX);
  }
}
