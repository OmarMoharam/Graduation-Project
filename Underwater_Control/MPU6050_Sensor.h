#ifndef MPU6050_SENSOR_H
#define MPU6050_SENSOR_H

#include <MPU6050_tockn.h>
#include <Wire.h>

class MPU6050_Sensor {
public:
  MPU6050_Sensor();
  void begin();
  void enableI2CBypass();
  void calibrate();
  void update();
  float getRoll();
  float getPitch();

private:
  MPU6050 mpu6050;
  float roll, pitch;
};

#endif