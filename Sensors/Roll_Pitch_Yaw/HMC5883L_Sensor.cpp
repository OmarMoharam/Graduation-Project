#include "HMC5883L_Sensor.h"

HMC5883L_Sensor::HMC5883L_Sensor() : mag(Adafruit_HMC5883_Unified(12345)) {}

void HMC5883L_Sensor::begin() {
  if (!mag.begin()) {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    while (1);
  }
}

void HMC5883L_Sensor::calibrate() {
  // Replace these with your calibration values
  offsetX = 0; // Hard iron offset for X
  offsetY = 0; // Hard iron offset for Y
  offsetZ = 0; // Hard iron offset for Z
  scaleX = 1;  // Soft iron scale for X
  scaleY = 1;  // Soft iron scale for Y
  scaleZ = 1;  // Soft iron scale for Z
}

void HMC5883L_Sensor::update() {
  sensors_event_t event;
  mag.getEvent(&event);

  // Apply calibration
  float calibratedX = (event.magnetic.x - offsetX) / scaleX;
  float calibratedY = (event.magnetic.y - offsetY) / scaleY;
  float calibratedZ = (event.magnetic.z - offsetZ) / scaleZ;

  // Calculate yaw (heading)
  yaw = atan2(calibratedY, calibratedX);
  if (yaw < 0) yaw += 2 * PI; // Normalize to 0-360 degrees
  yaw = yaw * 180 / PI; // Convert to degrees
}

float HMC5883L_Sensor::getYaw() {
  return yaw;
}