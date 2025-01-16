#include "MPU6050_Sensor.h"

MPU6050_Sensor::MPU6050_Sensor() : mpu6050(Wire) {}

void MPU6050_Sensor::begin() {
  Wire.begin();
  mpu6050.begin();
  enableI2CBypass(); // Enable I2C bypass mode
  mpu6050.calcGyroOffsets(true); // Perform gyroscope calibration
}

void MPU6050_Sensor::enableI2CBypass() {
  Wire.beginTransmission(0x68); // MPU6050 I2C address
  Wire.write(0x37);             // Register for INT_PIN_CFG
  Wire.write(0x02);             // Enable bypass mode (set bit 1)
  Wire.endTransmission();
}

void MPU6050_Sensor::update() {
  mpu6050.update();
  roll = mpu6050.getAngleX(); // Roll
  pitch = mpu6050.getAngleY(); // Pitch
}

float MPU6050_Sensor::getRoll() {
  return roll;
}

float MPU6050_Sensor::getPitch() {
  return pitch;
}